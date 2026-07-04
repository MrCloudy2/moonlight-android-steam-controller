package com.limelight.binding.input.driver;

import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.os.SystemClock;

import com.limelight.LimeLog;
import com.limelight.nvstream.input.ControllerPacket;
import com.limelight.nvstream.jni.MoonBridge;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Driver for the 2nd generation Valve Steam Controller ("Triton", 2026),
 * connected over USB or the Controller Puck / Nereid wireless dongles.
 *
 * The controller boots in "lizard mode" where each controller slot emulates
 * a keyboard/mouse (KEYBOARD | MOUSE | STYLUS input devices; no gamepad).
 * Disabling lizard mode is a single setting write, but a firmware watchdog
 * re-enables it, so the disable must be re-sent periodically.
 *
 * The Proteus puck exposes 2 CDC interfaces and 5 HID interfaces, and
 * re-enumerates between PIDs (0x1304/0x1302/0x1007) depending on link
 * state. A linked controller lands on one of the HID slot interfaces, and
 * wireless status broadcasts may arrive on other slots than the one
 * carrying controller state. We therefore read every HID interface in
 * parallel and treat whichever produces state reports as the controller.
 *
 * Protocol reference: SDL's SDL_hidapi_steam_triton.c (zlib license) and
 * Valve's published controller_constants.h / controller_structs.h.
 */
public class SteamControllerTriton extends AbstractController {

    private static final int VALVE_VID = 0x28de;
    private static final int PID_TRITON_WIRED = 0x1302;
    private static final int PID_PROTEUS_DONGLE = 0x1304; // Controller Puck
    private static final int PID_NEREID_DONGLE = 0x1305;
    private static final int PID_TRITON_ALT = 0x1007; // Transitional puck identity

    // The firmware watchdog re-enables lizard mode, so SDL re-disables it
    // every 3 seconds while the controller is in use
    private static final long LIZARD_REFRESH_INTERVAL_MS = 3000;

    // Rumble hardware safety timeout is ~50ms, so re-send active rumble every 40ms
    private static final long RUMBLE_RESEND_INTERVAL_MS = 40;

    // Feature report plumbing (report ID 1)
    private static final byte FEATURE_REPORT_ID = 1;
    private static final byte ID_SET_SETTINGS_VALUES = (byte) 0x87;
    private static final byte SETTING_LIZARD_MODE = 9;
    private static final short LIZARD_MODE_OFF = 0;

    // Input report IDs (first byte of an interrupt transfer)
    private static final byte ID_TRITON_CONTROLLER_STATE = 0x42;
    private static final byte ID_TRITON_CONTROLLER_STATE_BLE = 0x45;
    private static final byte ID_TRITON_CONTROLLER_STATE_TIMESTAMP = 0x47;
    private static final byte ID_TRITON_WIRELESS_STATUS_X = 0x46;
    private static final byte ID_TRITON_WIRELESS_STATUS = 0x79;

    private static final int WIRELESS_STATE_DISCONNECT = 1;
    private static final int WIRELESS_STATE_CONNECT = 2;

    // Output report for rumble
    private static final byte ID_OUT_REPORT_HAPTIC_RUMBLE = (byte) 0x80;

    // Button bits within the 32-bit button field of a state report
    private static final int TRITON_LBUTTON_A            = 0x00000001;
    private static final int TRITON_LBUTTON_B            = 0x00000002;
    private static final int TRITON_LBUTTON_X            = 0x00000004;
    private static final int TRITON_LBUTTON_Y            = 0x00000008;
    private static final int TRITON_HBUTTON_QAM          = 0x00000010;
    private static final int TRITON_LBUTTON_R3           = 0x00000020;
    private static final int TRITON_LBUTTON_VIEW         = 0x00000040;
    private static final int TRITON_HBUTTON_R4           = 0x00000080;
    private static final int TRITON_LBUTTON_R5           = 0x00000100;
    private static final int TRITON_LBUTTON_R            = 0x00000200;
    private static final int TRITON_LBUTTON_DPAD_DOWN    = 0x00000400;
    private static final int TRITON_LBUTTON_DPAD_RIGHT   = 0x00000800;
    private static final int TRITON_LBUTTON_DPAD_LEFT    = 0x00001000;
    private static final int TRITON_LBUTTON_DPAD_UP      = 0x00002000;
    private static final int TRITON_LBUTTON_MENU         = 0x00004000;
    private static final int TRITON_LBUTTON_L3           = 0x00008000;
    private static final int TRITON_LBUTTON_STEAM        = 0x00010000;
    private static final int TRITON_HBUTTON_L4           = 0x00020000;
    private static final int TRITON_LBUTTON_L5           = 0x00040000;
    private static final int TRITON_LBUTTON_L            = 0x00080000;
    private static final int TRITON_RIGHT_TOUCHPAD_CLICK = 0x00400000;
    private static final int TRITON_LEFT_TOUCHPAD_CLICK  = 0x04000000;

    private class Slot {
        final int ifaceNum;
        final UsbEndpoint inEndpt;
        final UsbEndpoint outEndpt;
        Thread thread;
        long lastLizardRefresh;
        int lizardFailures;
        final Set<Byte> seenReportIds = new HashSet<>();

        Slot(int ifaceNum, UsbEndpoint inEndpt, UsbEndpoint outEndpt) {
            this.ifaceNum = ifaceNum;
            this.inEndpt = inEndpt;
            this.outEndpt = outEndpt;
        }
    }

    private final UsbDevice device;
    private final UsbDeviceConnection connection;
    private final Object controlLock = new Object();

    private final List<Slot> slots = new ArrayList<>();
    private volatile Slot activeSlot;
    private volatile boolean stopped;
    private Thread announceThread;

    private volatile short lowFreqMotor, highFreqMotor;
    private long lastRumbleSend;

    public SteamControllerTriton(UsbDevice device, UsbDeviceConnection connection, int deviceId, UsbDriverListener listener) {
        super(deviceId, listener, device.getVendorId(), device.getProductId());
        this.device = device;
        this.connection = connection;
        this.type = MoonBridge.LI_CTYPE_UNKNOWN;
        this.capabilities = MoonBridge.LI_CCAP_ANALOG_TRIGGERS | MoonBridge.LI_CCAP_RUMBLE;
        this.buttonFlags =
                ControllerPacket.A_FLAG | ControllerPacket.B_FLAG | ControllerPacket.X_FLAG | ControllerPacket.Y_FLAG |
                        ControllerPacket.UP_FLAG | ControllerPacket.DOWN_FLAG | ControllerPacket.LEFT_FLAG | ControllerPacket.RIGHT_FLAG |
                        ControllerPacket.LB_FLAG | ControllerPacket.RB_FLAG |
                        ControllerPacket.LS_CLK_FLAG | ControllerPacket.RS_CLK_FLAG |
                        ControllerPacket.BACK_FLAG | ControllerPacket.PLAY_FLAG | ControllerPacket.SPECIAL_BUTTON_FLAG |
                        ControllerPacket.PADDLE1_FLAG | ControllerPacket.PADDLE2_FLAG |
                        ControllerPacket.PADDLE3_FLAG | ControllerPacket.PADDLE4_FLAG |
                        ControllerPacket.TOUCHPAD_FLAG | ControllerPacket.MISC_FLAG;
    }

    public static boolean canClaimDevice(UsbDevice device) {
        return device.getVendorId() == VALVE_VID &&
                (device.getProductId() == PID_TRITON_WIRED ||
                        device.getProductId() == PID_PROTEUS_DONGLE ||
                        device.getProductId() == PID_NEREID_DONGLE ||
                        device.getProductId() == PID_TRITON_ALT);
    }

    /**
     * Sends the setting that disables lizard mode on one slot interface.
     * A firmware watchdog re-enables it automatically, so this is re-sent
     * periodically. Restoring lizard mode on close is therefore unnecessary.
     */
    private void disableLizardMode(Slot slot) {
        // 64-byte feature report with report ID 1:
        // FeatureReportHeader { type, length } + ControllerSetting { num, value16 }
        byte[] buf = new byte[64];
        buf[0] = FEATURE_REPORT_ID;
        buf[1] = ID_SET_SETTINGS_VALUES;
        buf[2] = 3; // sizeof(ControllerSetting)
        buf[3] = SETTING_LIZARD_MODE;
        buf[4] = (byte) (LIZARD_MODE_OFF & 0xFF);
        buf[5] = (byte) ((LIZARD_MODE_OFF >> 8) & 0xFF);

        int res;
        synchronized (controlLock) {
            res = connection.controlTransfer(
                    UsbConstants.USB_TYPE_CLASS | 0x01 /* recipient: interface */, // 0x21
                    0x09, // SET_REPORT
                    0x0300 | FEATURE_REPORT_ID,
                    slot.ifaceNum,
                    buf, buf.length, 250);
        }
        if (res < 0) {
            // Slots without a linked controller reject this; log once and back off
            if (++slot.lizardFailures == 1) {
                LimeLog.warning("Triton: lizard disable failed on iface " + slot.ifaceNum);
            }
        }
        else {
            slot.lizardFailures = 0;
        }

        slot.lastLizardRefresh = SystemClock.uptimeMillis();
        if (slot.lizardFailures >= 3) {
            // Refresh reluctantly until this slot starts accepting again
            slot.lastLizardRefresh += 27000;
        }
    }

    private void sendRumble() {
        Slot slot = activeSlot;
        if (slot == null && !slots.isEmpty()) {
            slot = slots.get(0);
        }
        if (slot == null) {
            return;
        }

        // MsgHapticRumble: reportId, type, intensity16, left {speed16, gain8}, right {speed16, gain8}
        byte[] buf = new byte[10];
        buf[0] = ID_OUT_REPORT_HAPTIC_RUMBLE;
        buf[1] = 0; // type
        buf[2] = 0; // intensity
        buf[3] = 0;
        buf[4] = (byte) (lowFreqMotor & 0xFF);
        buf[5] = (byte) ((lowFreqMotor >> 8) & 0xFF);
        buf[6] = 0; // left gain
        buf[7] = (byte) (highFreqMotor & 0xFF);
        buf[8] = (byte) ((highFreqMotor >> 8) & 0xFF);
        buf[9] = 0; // right gain

        if (slot.outEndpt != null) {
            connection.bulkTransfer(slot.outEndpt, buf, buf.length, 100);
        }
        else {
            synchronized (controlLock) {
                connection.controlTransfer(
                        UsbConstants.USB_TYPE_CLASS | 0x01, // 0x21
                        0x09, // SET_REPORT
                        0x0200 | (ID_OUT_REPORT_HAPTIC_RUMBLE & 0xFF), // Output report
                        slot.ifaceNum,
                        buf, buf.length, 100);
            }
        }

        lastRumbleSend = SystemClock.uptimeMillis();
    }

    private void handleState(Slot slot, ByteBuffer buffer) {
        // TritonMTUNoQuat_t / TritonMTUNoQuat32TS_t (identical through the
        // stick fields, which is all we consume). Payload starts at offset 1
        // after the report ID: seq_num u8, buttons u32, triggers s16 x2,
        // left stick s16 x2, right stick s16 x2, then trackpads/IMU.
        if (buffer.limit() < 18) {
            return;
        }

        if (activeSlot != slot) {
            LimeLog.info("Triton: controller state active on iface " + slot.ifaceNum);
            activeSlot = slot;
        }

        int buttons = buffer.getInt(2);

        setButtonFlag(ControllerPacket.A_FLAG, buttons & TRITON_LBUTTON_A);
        setButtonFlag(ControllerPacket.B_FLAG, buttons & TRITON_LBUTTON_B);
        setButtonFlag(ControllerPacket.X_FLAG, buttons & TRITON_LBUTTON_X);
        setButtonFlag(ControllerPacket.Y_FLAG, buttons & TRITON_LBUTTON_Y);

        setButtonFlag(ControllerPacket.UP_FLAG, buttons & TRITON_LBUTTON_DPAD_UP);
        setButtonFlag(ControllerPacket.DOWN_FLAG, buttons & TRITON_LBUTTON_DPAD_DOWN);
        setButtonFlag(ControllerPacket.LEFT_FLAG, buttons & TRITON_LBUTTON_DPAD_LEFT);
        setButtonFlag(ControllerPacket.RIGHT_FLAG, buttons & TRITON_LBUTTON_DPAD_RIGHT);

        setButtonFlag(ControllerPacket.LB_FLAG, buttons & TRITON_LBUTTON_L);
        setButtonFlag(ControllerPacket.RB_FLAG, buttons & TRITON_LBUTTON_R);

        setButtonFlag(ControllerPacket.LS_CLK_FLAG, buttons & TRITON_LBUTTON_L3);
        setButtonFlag(ControllerPacket.RS_CLK_FLAG, buttons & TRITON_LBUTTON_R3);

        // Matching SDL: MENU -> Back/Select, VIEW -> Start
        setButtonFlag(ControllerPacket.BACK_FLAG, buttons & TRITON_LBUTTON_MENU);
        setButtonFlag(ControllerPacket.PLAY_FLAG, buttons & TRITON_LBUTTON_VIEW);
        setButtonFlag(ControllerPacket.SPECIAL_BUTTON_FLAG, buttons & TRITON_LBUTTON_STEAM);
        setButtonFlag(ControllerPacket.MISC_FLAG, buttons & TRITON_HBUTTON_QAM);

        setButtonFlag(ControllerPacket.PADDLE1_FLAG, buttons & TRITON_HBUTTON_R4);
        setButtonFlag(ControllerPacket.PADDLE2_FLAG, buttons & TRITON_HBUTTON_L4);
        setButtonFlag(ControllerPacket.PADDLE3_FLAG, buttons & TRITON_LBUTTON_R5);
        setButtonFlag(ControllerPacket.PADDLE4_FLAG, buttons & TRITON_LBUTTON_L5);

        setButtonFlag(ControllerPacket.TOUCHPAD_FLAG,
                buttons & (TRITON_LEFT_TOUCHPAD_CLICK | TRITON_RIGHT_TOUCHPAD_CLICK));

        // Triggers are 0..32767
        leftTrigger = buffer.getShort(6) / 32767.0f;
        rightTrigger = buffer.getShort(8) / 32767.0f;

        leftStickX = buffer.getShort(10) / 32767.0f;
        leftStickY = ~buffer.getShort(12) / 32767.0f;

        rightStickX = buffer.getShort(14) / 32767.0f;
        rightStickY = ~buffer.getShort(16) / 32767.0f;

        reportInput();
    }

    private void neutralizeState() {
        buttonFlags = 0;
        leftTrigger = rightTrigger = 0;
        leftStickX = leftStickY = 0;
        rightStickX = rightStickY = 0;
        reportInput();
    }

    private void handleRead(Slot slot, ByteBuffer buffer) {
        if (buffer.limit() < 2) {
            return;
        }

        byte reportId = buffer.get(0);

        // Log the first packet of each report type per slot to aid debugging
        if (slot.seenReportIds.size() < 10 && slot.seenReportIds.add(reportId)) {
            LimeLog.info("Triton: iface " + slot.ifaceNum + " first report 0x" +
                    Integer.toHexString(reportId & 0xFF) + " len " + buffer.limit());
        }

        switch (reportId) {
            case ID_TRITON_CONTROLLER_STATE:
            case ID_TRITON_CONTROLLER_STATE_BLE:
            case ID_TRITON_CONTROLLER_STATE_TIMESTAMP:
                handleState(slot, buffer);
                break;

            case ID_TRITON_WIRELESS_STATUS_X:
            case ID_TRITON_WIRELESS_STATUS:
                switch (buffer.get(1)) {
                    case WIRELESS_STATE_CONNECT:
                        LimeLog.info("Triton: wireless connect on iface " + slot.ifaceNum);
                        disableLizardMode(slot);
                        break;

                    case WIRELESS_STATE_DISCONNECT:
                        LimeLog.info("Triton: wireless disconnect on iface " + slot.ifaceNum);
                        if (activeSlot == slot) {
                            neutralizeState();
                        }
                        break;
                }
                break;
        }
    }

    private Thread createSlotThread(final Slot slot) {
        return new Thread() {
            public void run() {
                while (!isInterrupted() && !stopped) {
                    long now = SystemClock.uptimeMillis();

                    // The firmware watchdog will re-enable lizard mode unless
                    // we periodically re-disable it
                    if (now - slot.lastLizardRefresh >= LIZARD_REFRESH_INTERVAL_MS) {
                        disableLizardMode(slot);
                    }

                    // Keep active rumble alive past the hardware safety timeout
                    if (slot == activeSlot &&
                            (lowFreqMotor != 0 || highFreqMotor != 0) &&
                            now - lastRumbleSend >= RUMBLE_RESEND_INTERVAL_MS) {
                        sendRumble();
                    }

                    byte[] buffer = new byte[64];

                    // Idle slots will simply time out here over and over,
                    // which is fine. Fast failures indicate a device error.
                    long lastMillis = SystemClock.uptimeMillis();
                    int res = connection.bulkTransfer(slot.inEndpt, buffer, buffer.length, 1000);

                    if (res == 0) {
                        res = -1;
                    }

                    if (res == -1 && SystemClock.uptimeMillis() - lastMillis < 500) {
                        LimeLog.warning("Triton: I/O error on iface " + slot.ifaceNum);
                        SteamControllerTriton.this.stop();
                        break;
                    }

                    if (res > 0) {
                        handleRead(slot, ByteBuffer.wrap(buffer, 0, res).order(ByteOrder.LITTLE_ENDIAN));
                    }
                }
            }
        };
    }

    @Override
    public boolean start() {
        // Claim all interfaces, including any lizard mode keyboard/mouse
        // interfaces and the CDC interfaces on the puck. This detaches the
        // kernel HID driver so no mouse/keyboard input leaks through while
        // we own the controller.
        List<UsbInterface> hidIfaces = new ArrayList<>();
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface iface = device.getInterface(i);

            LimeLog.info("Triton: interface " + iface.getId() +
                    " class=" + iface.getInterfaceClass() +
                    " proto=" + iface.getInterfaceProtocol() +
                    " endpoints=" + iface.getEndpointCount());

            if (!connection.claimInterface(iface, true)) {
                LimeLog.warning("Failed to claim interface " + iface.getId());
                return false;
            }

            if (iface.getInterfaceClass() == UsbConstants.USB_CLASS_HID &&
                    iface.getInterfaceProtocol() == 0) {
                hidIfaces.add(iface);
            }
        }

        if (hidIfaces.isEmpty()) {
            LimeLog.warning("Triton: no raw HID interfaces found");
            return false;
        }

        // Each HID interface is a potential controller slot. Read all of
        // them; whichever produces controller state reports becomes the
        // active slot.
        for (UsbInterface iface : hidIfaces) {
            UsbEndpoint in = null, out = null;
            for (int i = 0; i < iface.getEndpointCount(); i++) {
                UsbEndpoint endpt = iface.getEndpoint(i);
                if (endpt.getDirection() == UsbConstants.USB_DIR_IN && in == null) {
                    in = endpt;
                }
                else if (endpt.getDirection() == UsbConstants.USB_DIR_OUT && out == null) {
                    out = endpt;
                }
            }

            if (in != null) {
                slots.add(new Slot(iface.getId(), in, out));
            }
        }

        if (slots.isEmpty()) {
            LimeLog.warning("Triton: no usable slot endpoints found");
            return false;
        }

        LimeLog.info("Triton: reading " + slots.size() + " slot interfaces on PID " +
                Integer.toHexString(device.getProductId()));

        for (Slot slot : slots) {
            disableLizardMode(slot);
            slot.thread = createSlotThread(slot);
            slot.thread.start();
        }

        // Delay for a moment before reporting the new gamepad, to allow any
        // lizard mode InputDevices to settle first.
        announceThread = new Thread() {
            public void run() {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    return;
                }
                notifyDeviceAdded();
            }
        };
        announceThread.start();

        return true;
    }

    @Override
    public void stop() {
        if (stopped) {
            return;
        }

        stopped = true;

        // Stop any rumble effect. Lizard mode restores itself via the
        // firmware watchdog a few seconds after we stop refreshing it.
        lowFreqMotor = highFreqMotor = 0;
        sendRumble();

        // Stop the threads
        if (announceThread != null) {
            announceThread.interrupt();
            announceThread = null;
        }
        for (Slot slot : slots) {
            if (slot.thread != null) {
                slot.thread.interrupt();
                slot.thread = null;
            }
        }

        // Close the USB connection
        connection.close();

        // Report the device removed
        notifyDeviceRemoved();
    }

    @Override
    public void rumble(short lowFreqMotor, short highFreqMotor) {
        this.lowFreqMotor = lowFreqMotor;
        this.highFreqMotor = highFreqMotor;
        sendRumble();
    }

    @Override
    public void rumbleTriggers(short leftTrigger, short rightTrigger) {
        // No trigger rumble hardware
    }
}
