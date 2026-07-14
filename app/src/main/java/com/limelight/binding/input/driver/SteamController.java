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

/**
 * Driver for the Valve Steam Controller (wired and wireless dongle).
 *
 * The Steam Controller boots in "lizard mode" where it emulates a USB
 * keyboard and mouse and exposes no usable gamepad. This driver claims the
 * raw HID controller interface, sends the feature reports that disable
 * lizard mode, and parses the raw input reports directly.
 *
 * Protocol reference: SDL's SDL_hidapi_steam.c (zlib license) and Valve's
 * published controller_constants.h / controller_structs.h.
 */
public class SteamController extends AbstractController {

    private static final int VALVE_VID = 0x28de;
    private static final int PID_WIRED = 0x1102;
    private static final int PID_DONGLE = 0x1142;

    // bInterfaceNumber of the raw controller interface. The remaining
    // interfaces are the lizard mode keyboard/mouse (and, on the dongle,
    // additional controller slots 2-4 which we don't drive yet).
    private static final int WIRED_CONTROLLER_INTERFACE = 2;
    private static final int DONGLE_CONTROLLER_INTERFACE = 1;

    // Feature report message IDs
    private static final byte ID_CLEAR_DIGITAL_MAPPINGS = (byte) 0x81;
    private static final byte ID_GET_ATTRIBUTES_VALUES = (byte) 0x83;
    private static final byte ID_SET_DEFAULT_DIGITAL_MAPPINGS = (byte) 0x85;
    private static final byte ID_SET_SETTINGS_VALUES = (byte) 0x87;
    private static final byte ID_LOAD_DEFAULT_SETTINGS = (byte) 0x8E;

    // Settings (index) and values for ID_SET_SETTINGS_VALUES
    private static final byte SETTING_LEFT_TRACKPAD_MODE = 7;
    private static final byte SETTING_RIGHT_TRACKPAD_MODE = 8;
    private static final byte SETTING_SMOOTH_ABSOLUTE_MOUSE = 24;
    private static final byte SETTING_WIRELESS_PACKET_VERSION = 49;
    private static final byte TRACKPAD_ABSOLUTE_MOUSE = 0;
    private static final byte TRACKPAD_NONE = 7;

    // Input report types (header byte 2)
    private static final int REPORT_VERSION = 1;
    private static final int ID_CONTROLLER_STATE = 1;
    private static final int ID_CONTROLLER_WIRELESS = 3;

    // Wireless event types for ID_CONTROLLER_WIRELESS reports
    private static final int WIRELESS_DISCONNECTED = 1;
    private static final int WIRELESS_ESTABLISHED = 2;

    // Button bits within the 64-bit button field of a state report
    private static final long STEAM_RIGHT_TRIGGER_MASK           = 0x00000001;
    private static final long STEAM_LEFT_TRIGGER_MASK            = 0x00000002;
    private static final long STEAM_RIGHT_BUMPER_MASK            = 0x00000004;
    private static final long STEAM_LEFT_BUMPER_MASK             = 0x00000008;
    private static final long STEAM_BUTTON_NORTH_MASK            = 0x00000010; // Y
    private static final long STEAM_BUTTON_EAST_MASK             = 0x00000020; // B
    private static final long STEAM_BUTTON_WEST_MASK             = 0x00000040; // X
    private static final long STEAM_BUTTON_SOUTH_MASK            = 0x00000080; // A
    private static final long STEAM_DPAD_UP_MASK                 = 0x00000100;
    private static final long STEAM_DPAD_RIGHT_MASK              = 0x00000200;
    private static final long STEAM_DPAD_LEFT_MASK               = 0x00000400;
    private static final long STEAM_DPAD_DOWN_MASK               = 0x00000800;
    private static final long STEAM_BUTTON_MENU_MASK             = 0x00001000; // Select
    private static final long STEAM_BUTTON_STEAM_MASK            = 0x00002000; // Guide
    private static final long STEAM_BUTTON_ESCAPE_MASK           = 0x00004000; // Start
    private static final long STEAM_BUTTON_BACK_LEFT_MASK        = 0x00008000; // Left grip
    private static final long STEAM_BUTTON_BACK_RIGHT_MASK       = 0x00010000; // Right grip
    private static final long STEAM_BUTTON_LEFTPAD_CLICKED_MASK  = 0x00020000;
    private static final long STEAM_BUTTON_RIGHTPAD_CLICKED_MASK = 0x00040000;
    private static final long STEAM_LEFTPAD_FINGERDOWN_MASK      = 0x00080000;
    private static final long STEAM_RIGHTPAD_FINGERDOWN_MASK     = 0x00100000;
    private static final long STEAM_JOYSTICK_BUTTON_MASK         = 0x00400000;
    private static final long STEAM_LEFTPAD_AND_JOYSTICK_MASK    = 0x00800000;

    // Analog trigger full-pull value: (0xFF << 7) | 0xFF clamped by firmware
    private static final float TRIGGER_MAX_ANALOG = 26620.0f;

    private final UsbDevice device;
    private final UsbDeviceConnection connection;
    private final int controllerIfaceNum;

    private Thread inputThread;
    private boolean stopped;
    private UsbEndpoint inEndpt;

    private int lastPacketNum;

    public SteamController(UsbDevice device, UsbDeviceConnection connection, int deviceId, UsbDriverListener listener,
                           String emulationMode) {
        super(deviceId, listener, device.getVendorId(), device.getProductId());
        this.device = device;
        this.connection = connection;
        this.controllerIfaceNum = (device.getProductId() == PID_DONGLE) ?
                DONGLE_CONTROLLER_INTERFACE : WIRED_CONTROLLER_INTERFACE;

        // The emulation mode selects what kind of controller the host sees
        switch (emulationMode != null ? emulationMode : "auto") {
            case "xbox":
                this.type = MoonBridge.LI_CTYPE_XBOX;
                break;
            case "ps":
                this.type = MoonBridge.LI_CTYPE_PS;
                break;
            case "nintendo":
                this.type = MoonBridge.LI_CTYPE_NINTENDO;
                break;
            default:
                this.type = MoonBridge.LI_CTYPE_UNKNOWN;
                break;
        }
        this.capabilities = MoonBridge.LI_CCAP_ANALOG_TRIGGERS;
        this.buttonFlags =
                ControllerPacket.A_FLAG | ControllerPacket.B_FLAG | ControllerPacket.X_FLAG | ControllerPacket.Y_FLAG |
                        ControllerPacket.UP_FLAG | ControllerPacket.DOWN_FLAG | ControllerPacket.LEFT_FLAG | ControllerPacket.RIGHT_FLAG |
                        ControllerPacket.LB_FLAG | ControllerPacket.RB_FLAG |
                        ControllerPacket.LS_CLK_FLAG | ControllerPacket.RS_CLK_FLAG |
                        ControllerPacket.BACK_FLAG | ControllerPacket.PLAY_FLAG | ControllerPacket.SPECIAL_BUTTON_FLAG |
                        ControllerPacket.PADDLE1_FLAG | ControllerPacket.PADDLE2_FLAG;
    }

    public static boolean canClaimDevice(UsbDevice device) {
        return device.getVendorId() == VALVE_VID &&
                (device.getProductId() == PID_WIRED || device.getProductId() == PID_DONGLE);
    }

    //
    // HID feature report plumbing. The Steam Controller firmware requires
    // 64-byte feature reports with report ID 0 (so no ID byte on the wire),
    // sent as class-specific control transfers against the controller interface.
    //

    private boolean setFeatureReport(byte[] report) {
        // The dongle radio occasionally NAKs requests while it's busy, so
        // retry a few times like SDL's radio retransmission workaround.
        for (int i = 0; i < 20; i++) {
            int res = connection.controlTransfer(
                    UsbConstants.USB_TYPE_CLASS | 0x01 /* recipient: interface */, // 0x21
                    0x09, // SET_REPORT
                    0x0300, // Feature report, report ID 0
                    controllerIfaceNum,
                    report, report.length, 2000);
            if (res >= 0) {
                return true;
            }

            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        LimeLog.warning("Steam Controller: SET_REPORT failed");
        return false;
    }

    private int getFeatureReport(byte[] report) {
        for (int i = 0; i < 20; i++) {
            int res = connection.controlTransfer(
                    UsbConstants.USB_TYPE_CLASS | UsbConstants.USB_DIR_IN | 0x01, // 0xA1
                    0x01, // GET_REPORT
                    0x0300, // Feature report, report ID 0
                    controllerIfaceNum,
                    report, report.length, 2000);
            if (res >= 0) {
                return res;
            }

            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return -1;
            }
        }
        return -1;
    }

    private boolean readResponse(byte[] report, byte expectedId) {
        for (int i = 0; i < 10; i++) {
            if (getFeatureReport(report) < 0) {
                continue;
            }
            if (report[0] == expectedId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Takes the controller out of lizard mode: clears the digital
     * keyboard/mouse mappings and turns off trackpad mouse emulation.
     */
    private boolean disableLizardMode() {
        byte[] buf = new byte[64];

        // Probe the controller. On the dongle this fails or returns nothing
        // until a controller has actually connected to the radio.
        buf[0] = ID_GET_ATTRIBUTES_VALUES;
        if (!setFeatureReport(buf)) {
            return false;
        }
        if (!readResponse(buf, ID_GET_ATTRIBUTES_VALUES)) {
            LimeLog.warning("Steam Controller: no GET_ATTRIBUTES response");
            return false;
        }

        // Clear the digital mappings (keyboard/mouse button emulation)
        buf = new byte[64];
        buf[0] = ID_CLEAR_DIGITAL_MAPPINGS;
        if (!setFeatureReport(buf)) {
            return false;
        }

        // Reset settings to defaults before applying ours
        buf = new byte[64];
        buf[0] = ID_LOAD_DEFAULT_SETTINGS;
        buf[1] = 0;
        if (!setFeatureReport(buf)) {
            return false;
        }

        // Apply our settings: disable trackpad mouse emulation entirely
        buf = new byte[64];
        buf[0] = ID_SET_SETTINGS_VALUES;
        int nSettings = 0;
        nSettings = addSetting(buf, nSettings, SETTING_WIRELESS_PACKET_VERSION, 2);
        nSettings = addSetting(buf, nSettings, SETTING_LEFT_TRACKPAD_MODE, TRACKPAD_NONE);
        nSettings = addSetting(buf, nSettings, SETTING_RIGHT_TRACKPAD_MODE, TRACKPAD_NONE);
        nSettings = addSetting(buf, nSettings, SETTING_SMOOTH_ABSOLUTE_MOUSE, 0);
        buf[1] = (byte) (nSettings * 3);
        return setFeatureReport(buf);
    }

    /**
     * Restores lizard mode so the controller works as a mouse for the OS
     * again after we release it.
     */
    private void enableLizardMode() {
        byte[] buf = new byte[64];
        buf[0] = ID_SET_DEFAULT_DIGITAL_MAPPINGS;
        setFeatureReport(buf);

        buf = new byte[64];
        buf[0] = ID_LOAD_DEFAULT_SETTINGS;
        buf[1] = 0;
        setFeatureReport(buf);

        buf = new byte[64];
        buf[0] = ID_SET_SETTINGS_VALUES;
        int nSettings = addSetting(buf, 0, SETTING_RIGHT_TRACKPAD_MODE, TRACKPAD_ABSOLUTE_MOUSE);
        buf[1] = (byte) (nSettings * 3);
        setFeatureReport(buf);
    }

    private static int addSetting(byte[] buf, int nSettings, byte setting, int value) {
        buf[2 + nSettings * 3] = setting;
        buf[2 + nSettings * 3 + 1] = (byte) (value & 0xFF);
        buf[2 + nSettings * 3 + 2] = (byte) ((value >> 8) & 0xFF);
        return nSettings + 1;
    }

    private void handleState(ByteBuffer buffer) {
        // ValveControllerStatePacket_t
        if (buffer.limit() < 24) {
            LimeLog.severe("Steam Controller state read too small: " + buffer.limit());
            return;
        }

        int packetNum = buffer.getInt(4);
        if (packetNum == lastPacketNum) {
            return;
        }
        lastPacketNum = packetNum;

        long buttons = buffer.getLong(8);

        setButtonFlag(ControllerPacket.A_FLAG, (int) (buttons & STEAM_BUTTON_SOUTH_MASK));
        setButtonFlag(ControllerPacket.B_FLAG, (int) (buttons & STEAM_BUTTON_EAST_MASK));
        setButtonFlag(ControllerPacket.X_FLAG, (int) (buttons & STEAM_BUTTON_WEST_MASK));
        setButtonFlag(ControllerPacket.Y_FLAG, (int) (buttons & STEAM_BUTTON_NORTH_MASK));

        // The dpad bits are set by the firmware from left trackpad clicks
        setButtonFlag(ControllerPacket.UP_FLAG, (int) (buttons & STEAM_DPAD_UP_MASK));
        setButtonFlag(ControllerPacket.RIGHT_FLAG, (int) (buttons & STEAM_DPAD_RIGHT_MASK));
        setButtonFlag(ControllerPacket.LEFT_FLAG, (int) (buttons & STEAM_DPAD_LEFT_MASK));
        setButtonFlag(ControllerPacket.DOWN_FLAG, (int) (buttons & STEAM_DPAD_DOWN_MASK));

        setButtonFlag(ControllerPacket.LB_FLAG, (int) (buttons & STEAM_LEFT_BUMPER_MASK));
        setButtonFlag(ControllerPacket.RB_FLAG, (int) (buttons & STEAM_RIGHT_BUMPER_MASK));

        setButtonFlag(ControllerPacket.BACK_FLAG, (int) (buttons & STEAM_BUTTON_MENU_MASK));
        setButtonFlag(ControllerPacket.PLAY_FLAG, (int) (buttons & STEAM_BUTTON_ESCAPE_MASK));
        setButtonFlag(ControllerPacket.SPECIAL_BUTTON_FLAG, (int) (buttons & STEAM_BUTTON_STEAM_MASK));

        setButtonFlag(ControllerPacket.PADDLE1_FLAG, (int) (buttons & STEAM_BUTTON_BACK_LEFT_MASK));
        setButtonFlag(ControllerPacket.PADDLE2_FLAG, (int) (buttons & STEAM_BUTTON_BACK_RIGHT_MASK));

        setButtonFlag(ControllerPacket.LS_CLK_FLAG, (int) (buttons & STEAM_JOYSTICK_BUTTON_MASK));
        setButtonFlag(ControllerPacket.RS_CLK_FLAG, (int) (buttons & STEAM_BUTTON_RIGHTPAD_CLICKED_MASK));

        // Analog triggers are single bytes packed inside the button field
        leftTrigger = clamp((((buffer.get(11) & 0xFF) << 7) | (buffer.get(11) & 0xFF)) / TRIGGER_MAX_ANALOG);
        rightTrigger = clamp((((buffer.get(12) & 0xFF) << 7) | (buffer.get(12) & 0xFF)) / TRIGGER_MAX_ANALOG);

        // The "left pad" field is multiplexed between the analog stick and
        // the left trackpad. When a finger is on the trackpad, the packet
        // carries trackpad coordinates and the stick state is unchanged
        // (LEFTPAD_AND_JOYSTICK) or centered.
        if ((buttons & STEAM_LEFTPAD_FINGERDOWN_MASK) == 0) {
            leftStickX = buffer.getShort(16) / 32767.0f;
            leftStickY = ~buffer.getShort(18) / 32767.0f;
        }
        else if ((buttons & STEAM_LEFTPAD_AND_JOYSTICK_MASK) == 0) {
            leftStickX = 0;
            leftStickY = 0;
        }

        // Map the right trackpad to the right stick using the absolute touch
        // position as the stick deflection.
        if ((buttons & STEAM_RIGHTPAD_FINGERDOWN_MASK) != 0) {
            rightStickX = buffer.getShort(20) / 32767.0f;
            rightStickY = ~buffer.getShort(22) / 32767.0f;
        }
        else {
            rightStickX = 0;
            rightStickY = 0;
        }

        reportInput();
    }

    private void neutralizeState() {
        buttonFlags = 0;
        leftTrigger = rightTrigger = 0;
        leftStickX = leftStickY = 0;
        rightStickX = rightStickY = 0;
        reportInput();
    }

    private static float clamp(float val) {
        return Math.max(0.0f, Math.min(1.0f, val));
    }

    private void handleRead(ByteBuffer buffer) {
        if (buffer.remaining() < 5) {
            return;
        }

        if ((buffer.getShort(0) & 0xFFFF) != REPORT_VERSION) {
            return;
        }

        switch (buffer.get(2)) {
            case ID_CONTROLLER_STATE:
                handleState(buffer);
                break;

            case ID_CONTROLLER_WIRELESS:
                switch (buffer.get(4)) {
                    case WIRELESS_ESTABLISHED:
                        // A controller just connected to the dongle. Take it
                        // out of lizard mode.
                        LimeLog.info("Steam Controller connected to dongle");
                        disableLizardMode();
                        break;

                    case WIRELESS_DISCONNECTED:
                        LimeLog.info("Steam Controller disconnected from dongle");
                        neutralizeState();
                        break;
                }
                break;
        }
    }

    private Thread createInputThread() {
        return new Thread() {
            public void run() {
                try {
                    // Delay for a moment before reporting the new gamepad, to
                    // allow the lizard mode InputDevices to settle first.
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    return;
                }

                // Report that we're added _before_ reporting input
                notifyDeviceAdded();

                while (!isInterrupted() && !stopped) {
                    byte[] buffer = new byte[64];

                    int res;

                    do {
                        // Read the next input state packet
                        long lastMillis = SystemClock.uptimeMillis();
                        res = connection.bulkTransfer(inEndpt, buffer, buffer.length, 3000);

                        // If we get a zero length response, treat it as an error
                        if (res == 0) {
                            res = -1;
                        }

                        if (res == -1 && SystemClock.uptimeMillis() - lastMillis < 1000) {
                            LimeLog.warning("Detected device I/O error");
                            SteamController.this.stop();
                            break;
                        }
                    } while (res == -1 && !isInterrupted() && !stopped);

                    if (res == -1 || stopped) {
                        break;
                    }

                    handleRead(ByteBuffer.wrap(buffer, 0, res).order(ByteOrder.LITTLE_ENDIAN));
                }
            }
        };
    }

    @Override
    public boolean start() {
        // Claim all interfaces, including the lizard mode keyboard/mouse
        // interfaces. This detaches the kernel HID driver so no mouse input
        // leaks through while we own the controller.
        UsbInterface controllerIface = null;
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface iface = device.getInterface(i);

            if (!connection.claimInterface(iface, true)) {
                LimeLog.warning("Failed to claim interface " + iface.getId());
                return false;
            }

            if (iface.getId() == controllerIfaceNum) {
                controllerIface = iface;
            }
        }

        if (controllerIface == null) {
            LimeLog.warning("Steam Controller interface " + controllerIfaceNum + " not found");
            return false;
        }

        // Find the interrupt IN endpoint on the controller interface
        for (int i = 0; i < controllerIface.getEndpointCount(); i++) {
            UsbEndpoint endpt = controllerIface.getEndpoint(i);
            if (endpt.getDirection() == UsbConstants.USB_DIR_IN) {
                inEndpt = endpt;
                break;
            }
        }

        if (inEndpt == null) {
            LimeLog.warning("Missing required endpoint");
            return false;
        }

        // Disable lizard mode. This fails harmlessly on a dongle with no
        // controller connected yet; we retry when we see the connection
        // event for the controller.
        if (!disableLizardMode() && device.getProductId() == PID_WIRED) {
            return false;
        }

        // Start listening for controller input
        inputThread = createInputThread();
        inputThread.start();

        return true;
    }

    @Override
    public void stop() {
        if (stopped) {
            return;
        }

        stopped = true;

        // Restore lizard mode so the controller still works for OS
        // navigation after Moonlight releases it
        enableLizardMode();

        // Stop the input thread
        if (inputThread != null) {
            inputThread.interrupt();
            inputThread = null;
        }

        // Close the USB connection
        connection.close();

        // Report the device removed
        notifyDeviceRemoved();
    }

    @Override
    public void rumble(short lowFreqMotor, short highFreqMotor) {
        // The Steam Controller has no rumble motors, only trackpad haptic
        // actuators. Not implemented yet.
    }

    @Override
    public void rumbleTriggers(short leftTrigger, short rightTrigger) {
        // No trigger rumble hardware
    }
}
