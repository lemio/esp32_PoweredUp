/*
  PoweredUp.h - Control LEGO WeDo 2.0 and LEGO Powered Up (train hubs, BOOST, Remote
  Control) over Bluetooth from an ESP32.

  Make one PoweredUp object per LEGO device you want to talk to (a hub, a remote
  control, ...) - you can have more than one connected at the same time.
*/
#ifndef PoweredUp_h
#define PoweredUp_h

#include "Arduino.h"
#include "wedo_color_definitions.h"
#include "ble_functions.h"
#include <functional>

// --- WeDo 2.0 protocol constants -------------------------------------------------------
#define ID_MOTOR 1
#define ID_TILT_SENSOR 34   // 0x22
#define ID_DETECT_SENSOR 35 // 0x23
#define RANGE_10 0
#define RANGE_100 1
#define RANGE_RAW 2

// --- Which kind of LEGO BLE device to look for (PoweredUp's second constructor argument) ---
// Matches the "System Type + Device Number" byte LEGO devices advertise, so you can ask
// for e.g. "a Remote Control" specifically instead of "any supported LEGO device",
// which matters once more than one kind of LEGO device is turned on nearby.
enum LegoDeviceType : uint8_t {
  DEVICE_TYPE_ANY               = 0xFF, // no filtering - connect to the first supported device found
  DEVICE_TYPE_ANY_HUB           = 0xFE, // any hub (WeDo, Powered Up, BOOST, train, Duplo) but never a Remote Control -
                                         // useful when a sketch also has a separate PoweredUp instance specifically
                                         // targeting DEVICE_TYPE_POWERED_UP_REMOTE, so the hub slot can't accidentally
                                         // claim the physical remote before the remote's own slot does
  DEVICE_TYPE_WEDO_HUB          = 0x00,
  DEVICE_TYPE_DUPLO_TRAIN       = 0x20,
  DEVICE_TYPE_BOOST_HUB         = 0x40,
  DEVICE_TYPE_POWERED_UP_HUB    = 0x41, // train hub, city hub, etc.
  DEVICE_TYPE_POWERED_UP_REMOTE = 0x42,
};

// --- IO Type IDs: what a hub reports when you plug something into a port ----------------
// Printed to Serial automatically whenever a device attaches, so you can always see
// these values for yourself rather than trusting this list blindly.
//
// The IO_TYPE_MEDIUM_MOTOR..IO_TYPE_LIGHT2 entries are older, "passive" LEGO Power
// Functions-style devices (the hub identifies them by a resistor/voltage pattern on
// their two ID pins rather than the device announcing itself) - reported as "LPF2-..."
// names by the discovery feature. All verified live; all are single-mode (mode 0) and
// output-only (actuators) except IO_TYPE_TOUCH, which is input-only.
enum LegoIoType : uint16_t {
  IO_TYPE_TILT_SENSOR   = 0x0022, // verified live: train hub port A/B
  IO_TYPE_MOTION_SENSOR = 0x0023, // verified live: train hub port A/B ("detect sensor")
  IO_TYPE_VOLTAGE       = 0x0014, // verified live: Remote Control internal port
  IO_TYPE_CURRENT       = 0x0015, // verified live: train hub internal port ("CUR L"/"CUR S")
  IO_TYPE_RSSI          = 0x0038, // verified live: Remote Control internal port
  IO_TYPE_REMOTE_BUTTON = 0x0037, // verified live: Remote Control port A/B
  IO_TYPE_RGB_LIGHT     = 0x0017, // verified live: hub LED, port varies by hub model

  IO_TYPE_MEDIUM_MOTOR  = 0x0001, // "LPF2-MMOTOR", -100..100
  IO_TYPE_TRAIN_MOTOR   = 0x0002, // "LPF2-TRAIN", -100..100 - the classic LEGO train motor
  IO_TYPE_TURN          = 0x0003, // "LPF2-TURN", -100..100
  IO_TYPE_POWER         = 0x0004, // "LPF2-POWER", -100..100
  IO_TYPE_TOUCH         = 0x0005, // "LPF2-TOUCH", input-only, 0 or 1 (pressed/not)
  IO_TYPE_LARGE_MOTOR   = 0x0006, // "LPF2-LMOTOR", -100..100
  IO_TYPE_XMOTOR        = 0x0007, // "LPF2-XMOTOR", -100..100
  IO_TYPE_LIGHT         = 0x0008, // "LPF2-LIGHT", -10..10
  IO_TYPE_LIGHT2        = 0x000a, // "LPF2-LIGHT2", 0..1
};

// --- Mode numbers, per device type ------------------------------------------------------
// A "mode" picks which measurement/format you get from a sensor, or which format you
// send to an actuator. onButtonPressed()/onDistanceChanged()/onTiltChanged()/
// remoteButton() already pick a sensible mode for you - these are for monitorInput(),
// when you want a specific one.
//
// The tilt/motion/current tables below come from running this library's own port/mode
// discovery feature (the table it prints to Serial when something attaches) against
// real hardware - not guessed from documentation.

// Hub/Remote LED (IO_TYPE_RGB_LIGHT) - verified live.
enum LedMode : uint8_t {
  LED_MODE_COL = 0, // a LEGO_COLOR_* index, 0-10 (used by writeIndexColor())
  LED_MODE_RGB = 1, // 3 raw bytes 0-255 (used by writeRGB())
};

// Remote Control button (IO_TYPE_REMOTE_BUTTON) - mode shapes/ranges verified live.
enum RemoteButtonMode : uint8_t {
  RCKEY = 0, // 1 byte: up(+)=1, down(-)=-1 (0xff), stop(red)=127, released=0 - not used by remoteButton(), which uses KEYSD instead
  KEYA  = 1, // 1 byte - exact meaning not yet verified
  KEYR  = 2, // 1 byte - exact meaning not yet verified
  KEYD  = 3, // 1 byte, range 0-7 - exact meaning not yet verified
  KEYSD = 4, // 3 separate bytes, each 0 or 1 - shape verified, USED BY remoteButton()
};

// Voltage sensor (IO_TYPE_VOLTAGE) - both modes' ranges verified live. "L"/"S" per the
// device's own reported names - likely "long"/"short" averaging windows.
enum VoltageMode : uint8_t {
  VLT_L = 0, // 0-9600 mV
  VLT_S = 1, // 0-9600 mV
};

// Current sensor (IO_TYPE_CURRENT) - verified live (name/mode shape), IO Type ID itself
// not independently confirmed - see IO_TYPE_CURRENT above.
enum CurrentMode : uint8_t {
  CUR_L = 0, // 0-2444 mA
  CUR_S = 1, // 0-2444 mA
};

// Signal strength (IO_TYPE_RSSI) - verified live, single mode, -80 to -30 dBm.
enum RssiMode : uint8_t {
  RSSI_MODE = 0,
};

// Motion/"detect" sensor (IO_TYPE_MOTION_SENSOR) - verified live. Mode 0 (DETECT) is
// what onDistanceChanged() uses.
enum MotionSensorMode : uint8_t {
  DETECT_MODE = 0, // 0-10 on both protocols - onDistanceChanged() configures WeDo 2.0's
                   // RANGE_10 to match this LWP3-native range (see _wedoRangeFormatFor()
                   // in PoweredUp.cpp), since the range is a property of the sensor's
                   // UART mode, not which hub it's plugged into.
  COUNT_MODE  = 1, // 0-100, 32-bit counter
  MOTION_CAL_MODE = 2, // 0-1023 raw, 3 datasets
};

// Tilt sensor (IO_TYPE_TILT_SENSOR) - verified live, all 4 modes. ANGLE (mode 0) is
// what onTiltChanged() uses, confirmed to be degrees, not an arbitrary unit.
enum TiltSensorMode : uint8_t {
  ANGLE_MODE = 0, // 2 datasets, -45 to 45 degrees (x/y tilt angle) - used by onTiltChanged()
  TILT_MODE  = 1, // 1 dataset, 0-10 (a coarser "which way is it tilted" direction reading)
  CRASH_MODE = 2, // 3 datasets, 0-100 (bump/impact counters per axis)
  TILT_CAL_MODE = 3, // 3 datasets, -45 to 45 (calibration values)
};

// Raw callback shape used internally, and by the advanced escape hatches
// (monitorInput()/addNotificationHandler()) that don't have a more specific typed
// callback - a std::function rather than a bare function pointer so it (and everything
// built on top of it) can hold a capturing lambda.
typedef std::function<void(int8_t*, int)> RawInputHandler;

class PoweredUp; // forward declaration - the handle classes below only need a pointer/reference to it

// Returned by PoweredUp::port() - lets you target a specific port explicitly (rather
// than relying on auto-detection) and ask what's actually plugged into it, e.g.:
//   hub.port('A').onDistanceChanged(callback);
//   if (hub.port('A') == IO_TYPE_MOTION_SENSOR) { ... }
// Note: right after connect(), a port's attached device may not be known yet (its attach
// event hasn't arrived) - a comparison checked too early can read "nothing attached"
// even if something is physically plugged in. Comparisons are always safe to call, but
// most reliable once handleConnection() has run for a bit.
class PortHandle {
  public:
    void onDistanceChanged(std::function<void(int8_t distance)> callback);
    void onTiltChanged(std::function<void(int8_t x, int8_t y)> callback);
    bool operator==(uint16_t ioType) const;
    bool operator!=(uint16_t ioType) const { return !(*this == ioType); }

  private:
    friend class PoweredUp;
    PoweredUp* _owner = nullptr;
    int _port = 0; // raw, as given to port() - normalized fresh wherever it's used
};

// One button's press/release state - used for up, stop, and down on a RemoteButtonHandle.
class ButtonEdge {
  public:
    // repeatMs=0 (default) = fire once per press. Non-zero = also keep firing the
    // pressed-callback every repeatMs while held (keyboard-style repeat).
    void onPressed(std::function<void()> callback, uint16_t repeatMs = 0);
    void onReleased(std::function<void()> callback);

  private:
    friend class PoweredUp;
    std::function<void()> _onPressed;
    std::function<void()> _onReleased;
    uint16_t _repeatMs = 0;
    bool _held = false;
    unsigned long _nextRepeatAt = 0;
};

// Returned by PoweredUp::remoteButton() - a Remote Control's three buttons on one port,
// e.g. remote.remoteButton('A').up.onPressed(callback). While stop is held, up/down
// presses are suppressed (their release fires instead) - stop is a momentary/e-stop
// style button and takes priority, so stop.onPressed()'s repeatMs is simply never used.
class RemoteButtonHandle {
  public:
    ButtonEdge up, stop, down;

  private:
    friend class PoweredUp;
    bool _inUse = false;
    bool _portGiven = false;
    int _requestedPort = 0; // raw, as given to remoteButton()
};

class PoweredUp {
  public:
    // Connect to a LEGO device by name, by kind (see LegoDeviceType), or both. Leave
    // both blank to connect to the first supported LEGO device found.
    PoweredUp(const char* name = nullptr, LegoDeviceType deviceType = DEVICE_TYPE_ANY);

    // Starts looking for the device and waits until it's connected (or timeoutMs has
    // passed). Safe to call on more than one PoweredUp object in a row in setup().
    int connect(uint32_t timeoutMs = 30000);
    boolean connected();
    boolean ready(); // true once the last command you sent has finished

    // Call this every loop() - keeps the connection alive and lets background work
    // (reconnecting, re-subscribing sensors, discovering what's plugged in) happen.
    void handleConnection();

    // Advanced: send a raw command yourself. type is WEDO_OUTPUT (the default - most
    // commands) or WEDO_INPUT (a few WeDo-specific ones). Most people won't need this.
    void writeCommand(uint8_t* command, int size, int type = WEDO_OUTPUT);

    // --- Actuators ---
    // Drives whichever motor is plugged in - if more than one is attached, or none has
    // been seen yet, this guesses port A. Use the two-argument version below if you need
    // to be specific (e.g. two motors on one hub).
    void writeMotor(int speed);
    // port: 'A'/'B' or 1/2 - which port the motor is plugged into.
    void writeMotor(int port, int speed);
    // A built-in LEGO colour, 0-10 (see the LEGO_COLOR_* constants).
    void writeIndexColor(uint8_t color);
    // Mix your own colour: red/green/blue, each 0 (none) to 255 (full).
    void writeRGB(uint8_t red, uint8_t green, uint8_t blue);
    // WeDo hubs only - plays a tone on the hub's built-in piezo speaker.
    void writeSound(unsigned int frequency, unsigned int length);
    // Drives a plugged-in simple LEGO Power Functions light (IO_TYPE_LIGHT), wherever
    // it's plugged in: value is -100 (off/dim) to 100 (full brightness).
    void writeLight(int value);

    // --- WeDo 2.0 low-level (WeDo hubs only) ---
    void writePortDefinition(uint8_t port, uint8_t type, uint8_t mode, uint8_t format);

    // --- Buttons & sensors (WeDo 2.0 hubs AND Powered Up / BOOST / train hubs) ---
    // Hub's own physical button (the LEGO-logo button on the hub itself - not a Remote
    // Control's buttons). No port - there's only one.
    void onButtonPressed(std::function<void()> callback);
    void onButtonReleased(std::function<void()> callback);

    // Distance/proximity sensor. Finds it itself if no port is given; on a WeDo 2.0 hub,
    // if called before the attach event has arrived (e.g. right after connect()), this
    // assumes port A and corrects itself automatically once the real attach report comes
    // in - or target a port explicitly via port() below to skip the guess entirely.
    // Range: 0-10 on both WeDo 2.0 and Powered Up/BOOST hubs (see DETECT_MODE).
    void onDistanceChanged(std::function<void(int8_t distance)> callback);
    // Tilt sensor's angle (x/y degrees, -45 to 45). Same port rules as onDistanceChanged().
    void onTiltChanged(std::function<void(int8_t x, int8_t y)> callback);

    // Explicit port targeting and introspection - e.g. hub.port('A').onDistanceChanged(cb),
    // or if (hub.port('A') == IO_TYPE_MOTION_SENSOR) {...}. port: 'A'/'B' or 1/2.
    PortHandle& port(int portArg);

    // Remote Control's three buttons (up/stop/down) on one port, e.g.
    // remote.remoteButton('A').up.onPressed(callback). Finds the remote's buttons itself
    // if no port is given.
    RemoteButtonHandle& remoteButton();
    RemoteButtonHandle& remoteButton(int portArg);

    // Advanced: listen to a specific mode yourself - see the *Mode enums above, or the
    // "Port mode information" table this library prints to Serial for what's actually
    // plugged in. Most people should use onDistanceChanged()/onTiltChanged()/port()
    // instead. Powered Up / BOOST / train hubs only.
    void monitorInput(int port, RawInputHandler callback, uint8_t mode);

    // Stops listening to whatever's on this port (from any of the methods above,
    // including port()/remoteButton() handles targeting it).
    void stopMonitoring(int port);
    // Stops all monitoring on this connection at once (every port, every port()/
    // remoteButton() handle, plus the hub button).
    void stopMonitoring();

    // Escape hatch: replaces the library's own notification handling with your own,
    // given the raw bytes straight off the BLE characteristic (unsigned, unlike the
    // int8_t sensor values everything else above hands you).
    void addNotificationHandler(std::function<void(uint8_t*, int)> f);

  private:
    static const uint8_t MAX_SUBSCRIPTIONS = 8;
    static const uint8_t MAX_ATTACHED_DEVICES = 8;
    static const uint8_t MAX_PENDING_MONITORS = 4;
    static const uint8_t MAX_DISCOVERY_QUEUE = 8;
    static const uint8_t MAX_CANDIDATE_TYPES = 4;
    static const uint8_t MAX_PORTS = 8;                // port() handles
    static const uint8_t MAX_REMOTE_BUTTON_GROUPS = 4; // remoteButton() handles - hardware only ever needs 2 (port A/B)

    friend class PortHandle;
    friend class RemoteButtonHandle;

    // One port this object has subscribed to input from, via monitorInput() (or
    // onDistanceChanged()/onTiltChanged()/port()/remoteButton(), which all call it).
    struct PortSubscription {
      bool inUse = false;
      uint8_t port = 0;
      uint8_t mode = 0;
      RawInputHandler handler;
      bool reArmPending = false; // re-subscribe needed - the hub drops it whenever a port's device detaches
    };

    // Every port we've seen something attach to, and what it was (IO Type ID) - lets
    // onDistanceChanged()/onTiltChanged()/port() find/identify the right port themselves.
    struct AttachedDeviceInfo {
      bool inUse = false;
      uint8_t port = 0;
      uint16_t ioTypeId = 0;
    };

    // A monitor call waiting for a matching device to attach (either no port was given,
    // or the given port didn't have a match yet).
    struct PendingMonitor {
      bool waiting = false;
      bool portGiven = false;
      uint8_t requestedPort = 0;
      uint8_t mode = 0;
      RawInputHandler callback;
      uint16_t candidateTypes[MAX_CANDIDATE_TYPES] = {0, 0, 0, 0};
      uint8_t candidateCount = 0;
      const char* label = nullptr; // for the Serial message, e.g. "onDistanceChanged"
    };

    // WeDo 2.0 equivalent of PendingMonitor above - a port-less onDistanceChanged()/
    // onTiltChanged() call that guessed port A because nothing had attached yet, kept
    // around so it can be corrected once a real attach event confirms (or contradicts)
    // that guess. WeDo only has 2 external ports, so this never needs to be large.
    struct WedoPendingMonitor {
      bool waiting = false;
      int8_t port = -1; // -1 = whichever port it turns up on; 0/1 = only that port
      uint8_t deviceId = 0;
      RawInputHandler callback;
      const char* label = nullptr;
    };
    // Two ports, but more than one waiting subscription can be aimed at the same port
    // (e.g. a port('A') and a port('B') call for the same kind of sensor, plus a
    // port-less one), so this is deliberately larger than the port count.
    static const uint8_t MAX_WEDO_PENDING = 4;
    WedoPendingMonitor _wedoPending[MAX_WEDO_PENDING];

    BLESlot _slot = BLE_SLOT_INVALID;

    // Set by addNotificationHandler() to bypass all of the library's own dispatch below.
    std::function<void(uint8_t*, int)> _userNotificationOverride;

    // WeDo 2.0 per-port sensor config (WeDo hubs only ever have 2 external ports).
    uint8_t _wedoDevices[2] = {0, 0};
    RawInputHandler _wedoHandlers[2];

    // What's actually plugged into each WeDo 2.0 port, from the port-type
    // characteristic's attach/detach events (0 = nothing attached/unknown yet). Uses
    // the same device-id numbering as ID_TILT_SENSOR/ID_DETECT_SENSOR/IO_TYPE_LIGHT
    // etc. Kept separate from the LWP3 attached-device directory below since WeDo 2.0's
    // notification format and re-subscribe behaviour are different enough that mixing
    // them caused more confusion than it saved.
    uint8_t _wedoAttachedDevice[2] = {0, 0};

    PortSubscription _subscriptions[MAX_SUBSCRIPTIONS];
    AttachedDeviceInfo _attached[MAX_ATTACHED_DEVICES];
    PendingMonitor _pending[MAX_PENDING_MONITORS];
    PortHandle _ports[MAX_PORTS];
    RemoteButtonHandle _remoteButtons[MAX_REMOTE_BUTTON_GROUPS];

    // The hub's own physical button (onButtonPressed()/onButtonReleased()) isn't tied to
    // a port at all - it's a separate LWP3 message type (Hub Properties), so it gets its
    // own handlers and its own "resubscribe after reconnect" tracking instead of using
    // the port machinery.
    std::function<void()> _onButtonPressed;
    std::function<void()> _onButtonReleased;
    bool _wasConnectedForHubButton = false;

    // WeDo 2.0's per-port sensor config (writePortDefinition(), sent once by
    // _monitorWedoDevice()) has no protocol-level re-arm the way LWP3's port
    // subscriptions do - the hub silently stops reporting a configured sensor after any
    // reconnect unless we resend it ourselves. Same "resend once on the connected
    // transition" pattern as the hub button above.
    bool _wasConnectedForWedoDevices = false;

    // Hub LED state (LWP3). _ledPort defaults to the train/Powered Up hub's LED port,
    // but a Remote Control's LED lives on a different port number - _ledModePort tracks
    // which port _ledActiveMode was actually confirmed on, so if _ledPort changes (e.g.
    // the real port is only known once its attach event arrives, which can race with an
    // early writeIndexColor()/writeRGB() call right after connect()) the mode gets
    // resent on the new port instead of assuming it's already set there.
    uint8_t _ledPort = 0x32;
    int8_t _ledActiveMode = -1;
    int _ledModePort = -1;
    int16_t _lastRGB[3] = {-1, -1, -1};
    int16_t _lastIndexColor = -1;
    bool _wasConnected = false;

    // Hub LED mode cache (WeDo 2.0) - same idea as _ledActiveMode above, so writeRGB()/
    // writeIndexColor() can switch modes for you without a separate setRGBMode() call.
    int8_t _wedoLedModeActive = -1;

    // Port/Port Mode discovery state machine - see PoweredUp.cpp for how this works.
    uint8_t _discoveryQueue[MAX_DISCOVERY_QUEUE];
    uint16_t _discoveryQueueIoType[MAX_DISCOVERY_QUEUE];
    uint8_t _discoveryQueueCount = 0;
    uint8_t _discoveryStep = 0;
    uint8_t _discoveryPort = 0;
    uint16_t _discoveryIoTypeId = 0; // IO Type ID of the port currently being discovered, printed alongside its modes so results can be matched back to a device later
    uint8_t _discoveryTotalModes = 0;
    uint16_t _discoveryInputModes = 0;
    uint16_t _discoveryOutputModes = 0;
    uint8_t _discoveryMode = 0;
    bool _discoveryStepComplete = false;
    unsigned long _discoveryStepSentAt = 0;
    char _modeName[12];
    float _modeRawMin, _modeRawMax;
    float _modePctMin, _modePctMax;
    float _modeSiMin, _modeSiMax;
    char _modeSymbol[6];
    uint8_t _modeNumDatasets, _modeDatasetType, _modeFigures, _modeDecimals;
    bool _haveName, _haveRaw, _havePct, _haveSi, _haveSymbol, _haveValueFormat;

    static void _notifyTrampoline(void* context, uint8_t* data, int size, BLENotificationSource source);
    void _handleNotification(uint8_t* data, int size, BLENotificationSource source);
    void _handleLwp3Notification(uint8_t* data, int size);
    void _handleWedoNotification(uint8_t* data, int size);
    void _handleWedoPortTypeNotification(uint8_t* data, int size);

    uint8_t _normalizePort(int portArg);

    int _findSubscription(uint8_t port);
    int _allocSubscription(uint8_t port);
    uint8_t _findMotorPort();
    void _recordAttached(uint8_t port, uint16_t ioTypeId);
    int _findAttachedPort(const uint16_t* candidateTypes, uint8_t candidateCount);
    uint16_t _attachedIoType(uint8_t normalizedPort); // for PortHandle::operator==

    void _monitorWithFallback(int portArg, bool portGiven, const uint16_t* candidateTypes,
                               uint8_t candidateCount, const char* label, uint8_t mode,
                               RawInputHandler callback);
    void _monitorWedoDevice(int portArg, bool portGiven, uint8_t deviceId, const char* label,
                             RawInputHandler callback);
    // True if a WeDo port may be configured for deviceId: either the hub has reported
    // that exact device there, or it hasn't reported anything there yet. A port the hub
    // says holds something else is never available - reconfiguring it would make a
    // working sensor report under the wrong device's format.
    bool _wedoPortAvailable(uint8_t normalizedPort, uint8_t deviceId);
    void _addWedoPendingMonitor(int8_t port, uint8_t deviceId, RawInputHandler callback,
                                 const char* label);
    void _resolvePendingMonitors(uint8_t port, uint16_t ioTypeId);
    void _resolveWedoPendingMonitors(uint8_t port, uint8_t deviceId); // port: 1-based

    RemoteButtonHandle& _ensureRemoteButtonGroup(int portArg, bool portGiven);
    void _handleRemoteButtonRaw(RemoteButtonHandle& group, int8_t* value, int size);

    void _writeMotorRaw(uint8_t rawPort, int speed);
    void _writeLwpCommand(uint8_t port, uint8_t mode, const uint8_t* payload, uint8_t payloadSize);
    void _sendPortInputFormatSetup(uint8_t port, uint8_t mode, bool enabled = true);
    void _sendHubButtonSubscribe(bool enabled);
    void _ensureLwp3LedMode(uint8_t mode);
    void _ensureWedoLedMode(uint8_t mode);
    void _resetLedCacheOnReconnect();

    void _sendPortInformationRequest(uint8_t port, uint8_t infoType);
    void _sendPortModeInformationRequest(uint8_t port, uint8_t mode, uint8_t infoType);
    void _queuePortDiscovery(uint8_t port, uint16_t ioTypeId);
    void _startModeQuery();
    void _advanceDiscovery();
    void _printModeRow();
};

#endif
