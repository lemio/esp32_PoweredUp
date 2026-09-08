#include "PoweredUp.h"

// Forward-declared: defined near _monitorWedoDevice() below, used earlier by
// handleConnection()'s WeDo reconnect-resend loop.
static uint8_t _wedoRangeFormatFor(uint8_t deviceId);

#define LWP_IMMEDIATE_NO_ACK 0x10
#define LWP_PORT_OUTPUT_COMMAND 0x81
#define LWP_WRITE_DIRECT_MODE_DATA 0x51
#define LWP_INTERNAL_LED_PORT 0x32
#define LWP_HUB_ATTACHED_IO 0x04
#define LWP_PORT_INPUT_FORMAT_SETUP_SINGLE 0x41
#define LWP_PORT_VALUE_SINGLE 0x45
#define LWP_PORT_INFORMATION_REQUEST 0x21
#define LWP_PORT_INFORMATION 0x43
#define LWP_PORT_MODE_INFORMATION_REQUEST 0x22
#define LWP_PORT_MODE_INFORMATION 0x44

// Hub Properties (message type 0x01) - a separate mechanism from ports/modes above, used
// for things that belong to the hub itself rather than something plugged into it (its
// own physical button, battery, name, ...). Only the Button property is used so far.
#define LWP_HUB_PROPERTIES 0x01
#define HUB_PROPERTY_BUTTON 0x02
#define HUB_PROPERTY_OP_ENABLE_UPDATES 0x02
#define HUB_PROPERTY_OP_DISABLE_UPDATES 0x03
#define HUB_PROPERTY_OP_UPDATE 0x06

// Port Information Request "Information Type" values (see LWP3 docs, Port Information Request).
#define PORT_INFO_MODE_INFO 0x01

// Port Mode Information Request "Mode Information Type" values.
#define MODE_INFO_NAME 0x00
#define MODE_INFO_RAW 0x01
#define MODE_INFO_PCT 0x02
#define MODE_INFO_SI 0x03
#define MODE_INFO_SYMBOL 0x04
#define MODE_INFO_VALUE_FORMAT 0x80

enum DiscoveryStep {
  DISCOVERY_STEP_IDLE = 0,
  DISCOVERY_STEP_PORT_INFO,
  DISCOVERY_STEP_NAME,
  DISCOVERY_STEP_RAW,
  DISCOVERY_STEP_PCT,
  DISCOVERY_STEP_SI,
  DISCOVERY_STEP_SYMBOL,
  DISCOVERY_STEP_VALUE_FORMAT,
};

#define DISCOVERY_STEP_TIMEOUT_MS 500

// ---------------------------------------------------------------------------------------
// Construction / connection
// ---------------------------------------------------------------------------------------

void PoweredUp::_notifyTrampoline(void* context, uint8_t* data, int size, BLENotificationSource source) {
  static_cast<PoweredUp*>(context)->_handleNotification(data, size, source);
}

PoweredUp::PoweredUp(const char* name, LegoDeviceType deviceType) {
  _slot = bleAcquireSlot(name, (uint8_t)deviceType);
  bleAddNotificationHandler(_slot, _notifyTrampoline, this);
}

int PoweredUp::connect(uint32_t timeoutMs) {
  bleConnect(_slot, timeoutMs);
  // Reset the LED cache here, synchronously, rather than lazily the first time
  // writeIndexColor()/writeRGB() is called. Attach events (which report the LED's real
  // port - see _handleLwp3Notification) can arrive asynchronously right after connect()
  // returns, before the caller's first writeIndexColor()/writeRGB() call. If the cache
  // reset happened lazily inside that first call instead, it would stomp the port an
  // attach event had already corrected, sending the color to the wrong port.
  _resetLedCacheOnReconnect();
  return 1;
}

boolean PoweredUp::connected() {
  return bleConnected(_slot);
}

boolean PoweredUp::ready() {
  return bleReady(_slot);
}

void PoweredUp::handleConnection() {
  // Pumps every connection (not just this object's) - see bleHandleConnections().
  bleHandleConnections();

  // Same reasoning as the call in connect() - catch a reconnect's "just connected"
  // transition here, eagerly, every loop iteration, rather than lazily inside the next
  // writeIndexColor()/writeRGB() call, which could otherwise race against an attach
  // event that already corrected the LED port for the reconnected device.
  _resetLedCacheOnReconnect();

  // Re-arm any LWP3 port subscriptions flagged by the notification handler. Done here
  // (outside the BLE callback) since a GATT write from within the notification
  // callback exhausts the NimBLE stack's buffer pool.
  for (uint8_t i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (_subscriptions[i].inUse && _subscriptions[i].reArmPending) {
      _subscriptions[i].reArmPending = false;
      _sendPortInputFormatSetup(_subscriptions[i].port, _subscriptions[i].mode);
    }
  }

  // Advance the port mode discovery/print state machine, same reason as above.
  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    _advanceDiscovery();
  }

  // The hub button subscription isn't tied to a port, so it isn't covered by the re-arm
  // loop above - resend it once whenever a fresh connection is made.
  bool isConnected = bleConnected(_slot);
  if (isConnected && !_wasConnectedForHubButton && (_onButtonPressed || _onButtonReleased) &&
      bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    _sendHubButtonSubscribe(true);
  }
  _wasConnectedForHubButton = isConnected;

  // WeDo 2.0's per-port sensor config has no protocol-level re-arm (unlike LWP3's
  // PortSubscription.reArmPending) - resend writePortDefinition() for every configured
  // port once whenever a fresh connection is made, or the hub silently stops reporting
  // that sensor after any reconnect.
  if (isConnected && !_wasConnectedForWedoDevices && bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    for (uint8_t i = 0; i < 2; i++) {
      if (_wedoDevices[i] > 0) {
        writePortDefinition(i + 1, _wedoDevices[i], 0, _wedoRangeFormatFor(_wedoDevices[i]));
      }
    }
  }
  _wasConnectedForWedoDevices = isConnected;

  // Keyboard-style repeat-while-held for remoteButton()'s up/down (if repeatMs was
  // given) - checked here, never from inside the notification callback, same reasoning
  // as the re-arm loop above. Already naturally suppressed while stop is held, since
  // _handleRemoteButtonRaw() forces _held false for up/down in that case.
  unsigned long now = millis();
  for (uint8_t i = 0; i < MAX_REMOTE_BUTTON_GROUPS; i++) {
    if (!_remoteButtons[i]._inUse) {
      continue;
    }
    ButtonEdge& up = _remoteButtons[i].up;
    ButtonEdge& down = _remoteButtons[i].down;
    if (up._held && up._repeatMs > 0 && now >= up._nextRepeatAt) {
      if (up._onPressed) up._onPressed();
      up._nextRepeatAt = now + up._repeatMs;
    }
    if (down._held && down._repeatMs > 0 && now >= down._nextRepeatAt) {
      if (down._onPressed) down._onPressed();
      down._nextRepeatAt = now + down._repeatMs;
    }
  }
}

void PoweredUp::onButtonPressed(std::function<void()> callback) {
  BLEHubProtocol protocol = bleProtocol(_slot);
  if (protocol != BLE_PROTOCOL_LWP3 && protocol != BLE_PROTOCOL_WEDO) {
    printf("onButtonPressed is only supported for WeDo 2.0 and LEGO Powered Up / BOOST / train hubs\n");
    return;
  }
  _onButtonPressed = callback;
  if (protocol == BLE_PROTOCOL_LWP3) {
    _sendHubButtonSubscribe(true);
  }
  // WeDo's button characteristic is already subscribed to at connect time (see
  // ble_functions.cpp) - no separate enable command needed, unlike LWP3's Hub
  // Properties message.
}

void PoweredUp::onButtonReleased(std::function<void()> callback) {
  BLEHubProtocol protocol = bleProtocol(_slot);
  if (protocol != BLE_PROTOCOL_LWP3 && protocol != BLE_PROTOCOL_WEDO) {
    printf("onButtonReleased is only supported for WeDo 2.0 and LEGO Powered Up / BOOST / train hubs\n");
    return;
  }
  _onButtonReleased = callback;
  if (protocol == BLE_PROTOCOL_LWP3) {
    _sendHubButtonSubscribe(true);
  }
}

// ---------------------------------------------------------------------------------------
// Low-level writes
// ---------------------------------------------------------------------------------------

void PoweredUp::writeCommand(uint8_t* command, int size, int type) {
  // Give the previous write a moment to finish (up to 100ms) so two commands sent back
  // to back don't race each other.
  for (int i = 0; i < 20 && !ready(); i++) {
    delay(5);
  }
  bleWriteCommand(_slot, type, command, size);
}

void PoweredUp::_writeLwpCommand(uint8_t port, uint8_t mode, const uint8_t* payload, uint8_t payloadSize) {
  const uint8_t header_size = 7;
  uint8_t command[header_size + payloadSize];

  command[0] = header_size + payloadSize;
  command[1] = 0x00;
  command[2] = LWP_PORT_OUTPUT_COMMAND;
  command[3] = port;
  command[4] = LWP_IMMEDIATE_NO_ACK;
  command[5] = LWP_WRITE_DIRECT_MODE_DATA;
  command[6] = mode;

  for (uint8_t i = 0; i < payloadSize; i++) {
    command[header_size + i] = payload[i];
  }

  bleWriteCommand(_slot, WEDO_OUTPUT, command, sizeof(command));
}

void PoweredUp::_sendPortInputFormatSetup(uint8_t port, uint8_t mode, bool enabled) {
  // Port Input Format Setup (Single): enable/disable notifications for this port/mode.
  // Delta interval of 1 means "notify on every change".
  uint8_t command[] = {0x0A, 0x00, LWP_PORT_INPUT_FORMAT_SETUP_SINGLE, port, mode,
                        0x01, 0x00, 0x00, 0x00, (uint8_t)(enabled ? 0x01 : 0x00)};
  bleWriteCommand(_slot, WEDO_OUTPUT, command, sizeof(command));
}

void PoweredUp::_sendHubButtonSubscribe(bool enabled) {
  uint8_t command[] = {0x05, 0x00, LWP_HUB_PROPERTIES, HUB_PROPERTY_BUTTON,
                        (uint8_t)(enabled ? HUB_PROPERTY_OP_ENABLE_UPDATES : HUB_PROPERTY_OP_DISABLE_UPDATES)};
  bleWriteCommand(_slot, WEDO_OUTPUT, command, sizeof(command));
}

// ---------------------------------------------------------------------------------------
// Port addressing
// ---------------------------------------------------------------------------------------

// Accepts a port as a letter ('A', 'B', ...) or a 1-based number (1, 2, ...) and always
// returns a 0-based index. Letters and small numbers never overlap in ASCII, so there's
// no ambiguity between the two spellings.
uint8_t PoweredUp::_normalizePort(int portArg) {
  if ((portArg >= 'A' && portArg <= 'Z') || (portArg >= 'a' && portArg <= 'z')) {
    return (uint8_t)(toupper(portArg) - 'A');
  }
  if (portArg >= 1) {
    return (uint8_t)(portArg - 1);
  }
  return (uint8_t)portArg;
}

// ---------------------------------------------------------------------------------------
// Actuators
// ---------------------------------------------------------------------------------------

void PoweredUp::_writeMotorRaw(uint8_t rawPort, int speed) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    uint8_t speed_byte = static_cast<uint8_t>(speed);
    uint8_t payload[] = {speed_byte};
    _writeLwpCommand(rawPort, 0x00, payload, sizeof(payload));
    return;
  }

  // conversion from int (both pos and neg) to unsigned 8 bit int
  uint8_t speed_byte = speed;
  uint8_t command[] = {(uint8_t)(rawPort + 1), 0x01, 0x01, speed_byte};
  writeCommand(command, sizeof(command));
}

void PoweredUp::writeMotor(int port, int speed) {
  _writeMotorRaw(_normalizePort(port), speed);
}

uint8_t PoweredUp::_findMotorPort() {
  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    for (uint8_t i = 0; i < 2; i++) {
      if (_wedoAttachedDevice[i] == ID_MOTOR) {
        return i;
      }
    }
    return 0; // hasn't reported a motor yet - guess port A
  }

  static const uint16_t motorTypes[] = {IO_TYPE_TRAIN_MOTOR, IO_TYPE_MEDIUM_MOTOR,
                                         IO_TYPE_LARGE_MOTOR, IO_TYPE_XMOTOR};
  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    int found = _findAttachedPort(motorTypes, 4);
    if (found >= 0) {
      return (uint8_t)found;
    }
  }
  return 0; // nothing confirmed - guess port A
}

void PoweredUp::writeMotor(int speed) {
  _writeMotorRaw(_findMotorPort(), speed);
}

void PoweredUp::writeLight(int value) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    for (uint8_t i = 0; i < 2; i++) {
      if (_wedoAttachedDevice[i] != IO_TYPE_LIGHT) {
        continue;
      }
      // WeDo 2.0's write command for a simple single-channel output (motor or light)
      // is the same shape either way - both are just "set this port's PWM output".
      int8_t scaled = (int8_t)(value / 10); // match LPF2-LIGHT's -10..10 raw range
      uint8_t command[] = {(uint8_t)(i + 1), 0x01, 0x01, (uint8_t)scaled};
      writeCommand(command, sizeof(command));
      return;
    }
    return; // no LPF2-LIGHT seen yet - nothing to write to
  }

  if (bleProtocol(_slot) != BLE_PROTOCOL_LWP3) {
    printf("writeLight is only supported for WeDo 2.0 and LEGO Powered Up / BOOST / train hubs\n");
    return;
  }
  uint16_t lightType = IO_TYPE_LIGHT;
  int port = _findAttachedPort(&lightType, 1);
  if (port < 0) {
    return; // no LPF2-LIGHT seen yet - nothing to write to
  }
  int8_t raw = (int8_t)(value / 10); // LPF2-LIGHT's actual raw range is -10..10
  uint8_t payload[] = {(uint8_t)raw};
  _writeLwpCommand((uint8_t)port, 0x00, payload, sizeof(payload));
}

// The hub LED only acts on WriteDirectModeData for whichever mode it's currently
// switched into - a Port Input Format Setup has to select that mode first, or writes to
// the other mode are silently ignored. These two helpers track the active mode so
// writeRGB()/writeIndexColor() only resend the switch when it actually changes, and so
// callers never need a separate "set mode" call of their own.
void PoweredUp::_ensureLwp3LedMode(uint8_t mode) {
  if (_ledActiveMode != mode || _ledModePort != _ledPort) {
    _sendPortInputFormatSetup(_ledPort, mode);
    _ledActiveMode = mode;
    _ledModePort = _ledPort;
  }
}

void PoweredUp::_ensureWedoLedMode(uint8_t mode) {
  if (_wedoLedModeActive == mode) {
    return;
  }
  // Confirmed live against real hardware to match the documented "RGB Absolute"/"RGB
  // Discrete" mode-select commands exactly (cpseager/WeDo2-BLE-Protocol,
  // wedo2_summary.txt) - format/unit is 0x00 for both, not just absolute mode.
  writePortDefinition(0x06, 0x17, mode == 0x00 ? 0x00 : 0x01, 0x00);
  _wedoLedModeActive = mode;
}

void PoweredUp::_resetLedCacheOnReconnect() {
  bool isConnected = bleConnected(_slot);
  if (isConnected && !_wasConnected) {
    _ledPort = LWP_INTERNAL_LED_PORT;
    _ledActiveMode = -1;
    _ledModePort = -1;
    _lastRGB[0] = _lastRGB[1] = _lastRGB[2] = -1;
    _lastIndexColor = -1;
    _wedoLedModeActive = -1;
  }
  _wasConnected = isConnected;
}

void PoweredUp::writeIndexColor(uint8_t color) {
  _resetLedCacheOnReconnect();

  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    if (_ledActiveMode == 0x00 && _lastIndexColor == color) {
      return; // unchanged since last write - avoid flooding the hub with redundant writes
    }
    _ensureLwp3LedMode(0x00);
    uint8_t payload[] = {color};
    _writeLwpCommand(_ledPort, 0x00, payload, sizeof(payload));
    _lastIndexColor = color;
    return;
  }

  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    if (_wedoLedModeActive == 0x00 && _lastIndexColor == color) {
      return; // unchanged since last write - avoid flooding the hub with redundant writes
    }
    _ensureWedoLedMode(0x00);
  }
  // From http://ofalcao.pt/blog/2016/wedo-2-0-colors-with-python
  uint8_t command[] = {0x06, 0x04, 0x01, color};
  writeCommand(command, sizeof(command));
  _lastIndexColor = color;
}

void PoweredUp::writeRGB(uint8_t red, uint8_t green, uint8_t blue) {
  _resetLedCacheOnReconnect();

  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    if (_ledActiveMode == 0x01 && _lastRGB[0] == red && _lastRGB[1] == green && _lastRGB[2] == blue) {
      return; // unchanged since last write - avoid flooding the hub with redundant writes
    }
    _ensureLwp3LedMode(0x01);
    uint8_t payload[] = {red, green, blue};
    _writeLwpCommand(_ledPort, 0x01, payload, sizeof(payload));
    _lastRGB[0] = red;
    _lastRGB[1] = green;
    _lastRGB[2] = blue;
    return;
  }

  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    if (_wedoLedModeActive == 0x01 && _lastRGB[0] == red && _lastRGB[1] == green && _lastRGB[2] == blue) {
      return; // unchanged since last write - avoid flooding the hub with redundant writes
    }
    _ensureWedoLedMode(0x01);
  }
  uint8_t command[] = {0x06, 0x04, 0x03, red, green, blue};
  writeCommand(command, sizeof(command));
  _lastRGB[0] = red;
  _lastRGB[1] = green;
  _lastRGB[2] = blue;
}

void PoweredUp::writeSound(unsigned int frequency, unsigned int length) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    printf("writeSound is only supported for WEDO hubs\n");
    return;
  }

  // From https://github.com/vheun/wedo2/blob/master/index.js (setSound)
  uint8_t command[] = {
    0x05,
    0x02,
    0x04,
    uint8_t((frequency >> (8 * 0)) & 0xff),
    uint8_t((frequency >> (8 * 1)) & 0xff),
    uint8_t((length >> (8 * 0)) & 0xff),
    uint8_t((length >> (8 * 1)) & 0xff)
  };
  writeCommand(command, sizeof(command));
}

// ---------------------------------------------------------------------------------------
// WeDo 2.0 low-level
// ---------------------------------------------------------------------------------------

void PoweredUp::writePortDefinition(uint8_t port, uint8_t type, uint8_t mode, uint8_t format) {
  if (bleProtocol(_slot) != BLE_PROTOCOL_WEDO) {
    printf("writePortDefinition is only supported for WEDO hubs\n");
    return;
  }

  uint8_t command[] = {0x01, 0x02, port, type, mode, 0x01, 0x00, 0x00, 0x00, format, 0x01};
  writeCommand(command, sizeof(command), WEDO_INPUT);
}

// The detect/distance sensor's raw UART range depends on the format byte, not the hub
// it's plugged into - RANGE_10 here matches the 0-10 range LWP3's DETECT_MODE already
// uses natively, so onDistanceChanged() reports the same scale on WeDo 2.0 and Powered
// Up/BOOST hubs alike. The tilt sensor's angle mode isn't affected by this byte the same
// way (RANGE_100 already verified live as the correct -45..45 degree reading), so it
// keeps the format that's been confirmed working.
static uint8_t _wedoRangeFormatFor(uint8_t deviceId) {
  return deviceId == ID_DETECT_SENSOR ? RANGE_10 : RANGE_100;
}

bool PoweredUp::_wedoPortAvailable(uint8_t normalizedPort, uint8_t deviceId) {
  if (normalizedPort >= 2) {
    return false;
  }
  return _wedoAttachedDevice[normalizedPort] == 0 ||
         _wedoAttachedDevice[normalizedPort] == deviceId;
}

// Records what the sketch asked for. Calling onTiltChanged() twice for the same port
// replaces the callback rather than filling up the table.
int PoweredUp::_addWedoSubscription(int8_t port, uint8_t deviceId, RawInputHandler callback,
                                     const char* label) {
  int freeSlot = -1;
  for (uint8_t i = 0; i < MAX_WEDO_SUBSCRIPTIONS; i++) {
    if (_wedoSubscriptions[i].inUse) {
      if (_wedoSubscriptions[i].port == port && _wedoSubscriptions[i].deviceId == deviceId) {
        _wedoSubscriptions[i].callback = callback;
        _wedoSubscriptions[i].label = label;
        return i;
      }
    } else if (freeSlot < 0) {
      freeSlot = i;
    }
  }
  if (freeSlot < 0) {
    printf("%s: no room for another WeDo subscription (max %d)\n", label, MAX_WEDO_SUBSCRIPTIONS);
    return -1;
  }
  _wedoSubscriptions[freeSlot].inUse = true;
  _wedoSubscriptions[freeSlot].port = port;
  _wedoSubscriptions[freeSlot].deviceId = deviceId;
  _wedoSubscriptions[freeSlot].callback = callback;
  _wedoSubscriptions[freeSlot].label = label;
  return freeSlot;
}

void PoweredUp::_configureWedoPort(uint8_t normalizedPort, WedoSubscription& sub) {
  // Safe to call while a notification callback is running (which is where _bindWedoPort()
  // reaches this from): writePortDefinition() goes through writeCommand()/bleWriteCommand(),
  // which queues the write and lets bleHandleConnections() send it from the main loop.
  writePortDefinition(normalizedPort + 1, sub.deviceId, 0, _wedoRangeFormatFor(sub.deviceId));
  _wedoDevices[normalizedPort] = sub.deviceId;
  _wedoHandlers[normalizedPort] = sub.callback;
  sub.boundPort = (int8_t)normalizedPort;
}

void PoweredUp::_bindWedoPort(uint8_t normalizedPort) {
  if (normalizedPort >= 2) {
    return;
  }
  uint8_t deviceId = _wedoAttachedDevice[normalizedPort];
  if (deviceId == 0) {
    return; // nothing plugged in - nothing to bind
  }

  int best = -1;
  for (uint8_t i = 0; i < MAX_WEDO_SUBSCRIPTIONS; i++) {
    WedoSubscription& s = _wedoSubscriptions[i];
    if (!s.inUse || s.deviceId != deviceId) {
      continue;
    }
    if (s.port >= 0 && s.port != (int8_t)normalizedPort) {
      continue; // named a different port - it keeps waiting for that one
    }
    if (s.port < 0 && s.boundPort >= 0 && s.boundPort != (int8_t)normalizedPort) {
      continue; // "wherever it turns up" already turned up somewhere, and is still there
    }
    // A subscription that named this port beats one that takes any port.
    if (best < 0 || (_wedoSubscriptions[best].port < 0 && s.port >= 0)) {
      best = i;
    }
  }
  if (best < 0) {
    return; // nothing subscribed to this kind of device
  }

  printf("%s: listening on port %c\n", _wedoSubscriptions[best].label, 'A' + normalizedPort);
  _configureWedoPort(normalizedPort, _wedoSubscriptions[best]);
}

void PoweredUp::_monitorWedoDevice(int portArg, bool portGiven, uint8_t deviceId, const char* label,
                                    RawInputHandler callback) {
  int8_t wanted = -1; // -1 = no particular port

  if (portGiven) {
    uint8_t p = _normalizePort(portArg);
    if (p >= 2) {
      printf("%s: WeDo 2.0 only has ports A and B - ignoring this call\n", label);
      return;
    }
    // A named port is taken literally. This used to fall back to the *other* port when
    // the device wasn't on the named one, which quietly handed this callback a port
    // another subscription was already using.
    wanted = (int8_t)p;
  }

  int idx = _addWedoSubscription(wanted, deviceId, callback, label);
  if (idx < 0) {
    return;
  }

  // Already know where it is? Bind it now.
  for (uint8_t p = 0; p < 2; p++) {
    if ((wanted < 0 || wanted == (int8_t)p) && _wedoAttachedDevice[p] == deviceId) {
      _configureWedoPort(p, _wedoSubscriptions[idx]);
      return;
    }
  }

  // Otherwise configure a port the hub hasn't claimed for something else, so a sketch
  // that subscribes before any attach event arrives still works. A port the hub says
  // holds a different device is never touched - reconfiguring it would make a working
  // sensor report under the wrong device's format. The subscription stays on file
  // either way, so the next attach event binds it wherever it really turns up.
  int8_t guess = -1;
  if (wanted >= 0) {
    if (_wedoPortAvailable((uint8_t)wanted, deviceId)) {
      guess = wanted;
    }
  } else {
    // Prefer a port no other subscription has claimed, so two port-less calls don't
    // pile onto the same one.
    for (uint8_t p = 0; p < 2 && guess < 0; p++) {
      if (_wedoAttachedDevice[p] == 0 && _wedoDevices[p] == 0) {
        guess = (int8_t)p;
      }
    }
    for (uint8_t p = 0; p < 2 && guess < 0; p++) {
      if (_wedoAttachedDevice[p] == 0) {
        guess = (int8_t)p;
      }
    }
  }

  if (guess < 0) {
    if (wanted >= 0) {
      printf("%s: port %c has a different device attached (type %d) - waiting until the "
             "expected device is plugged into that port\n", label, 'A' + wanted,
             _wedoAttachedDevice[wanted]);
    } else {
      printf("%s: both WeDo 2.0 ports already have other devices attached - waiting for "
             "the expected device to be plugged in\n", label);
    }
    return;
  }

  printf("%s: WeDo 2.0 hasn't reported this device yet - using port %c for now, will "
         "correct automatically once it attaches.\n", label, 'A' + guess);
  _configureWedoPort((uint8_t)guess, _wedoSubscriptions[idx]);
}

// ---------------------------------------------------------------------------------------
// LWP3 port subscriptions (monitorInput / onDistanceChanged / onTiltChanged / remoteButton)
// ---------------------------------------------------------------------------------------

int PoweredUp::_findSubscription(uint8_t port) {
  for (uint8_t i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (_subscriptions[i].inUse && _subscriptions[i].port == port) {
      return i;
    }
  }
  return -1;
}

int PoweredUp::_allocSubscription(uint8_t port) {
  int existing = _findSubscription(port);
  if (existing >= 0) {
    return existing;
  }
  for (uint8_t i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (!_subscriptions[i].inUse) {
      _subscriptions[i].inUse = true;
      _subscriptions[i].port = port;
      _subscriptions[i].reArmPending = false;
      return i;
    }
  }
  return -1;
}

void PoweredUp::monitorInput(int port, RawInputHandler callback, uint8_t mode) {
  if (bleProtocol(_slot) != BLE_PROTOCOL_LWP3) {
    printf("monitorInput is only supported for LEGO Powered Up / BOOST / train hubs and remotes\n");
    return;
  }

  uint8_t rawPort = _normalizePort(port);
  int idx = _allocSubscription(rawPort);
  if (idx < 0) {
    printf("monitorInput: no room for another subscription (max %d)\n", MAX_SUBSCRIPTIONS);
    return;
  }

  _subscriptions[idx].mode = mode;
  _subscriptions[idx].handler = callback;
  _subscriptions[idx].fromMonitor = false; // named its port outright - a detach won't drop it
  _sendPortInputFormatSetup(rawPort, mode);
}

// --- Attached-device directory -----------------------------------------------------

void PoweredUp::_recordAttached(uint8_t port, uint16_t ioTypeId) {
  int existing = -1;
  int freeSlot = -1;
  for (uint8_t i = 0; i < MAX_ATTACHED_DEVICES; i++) {
    if (_attached[i].inUse && _attached[i].port == port) {
      existing = i;
      break;
    }
    if (freeSlot < 0 && !_attached[i].inUse) {
      freeSlot = i;
    }
  }

  int idx = existing >= 0 ? existing : freeSlot;
  if (idx >= 0) {
    _attached[idx].inUse = true;
    _attached[idx].port = port;
    _attached[idx].ioTypeId = ioTypeId;
  }

  _bindLwp3Port(port, ioTypeId);
}

int PoweredUp::_findAttachedPort(const uint16_t* candidateTypes, uint8_t candidateCount) {
  for (uint8_t i = 0; i < MAX_ATTACHED_DEVICES; i++) {
    if (!_attached[i].inUse) {
      continue;
    }
    for (uint8_t c = 0; c < candidateCount; c++) {
      if (_attached[i].ioTypeId == candidateTypes[c]) {
        return _attached[i].port;
      }
    }
  }
  return -1;
}

// --- onDistanceChanged() / onTiltChanged() / remoteButton() (simple, with fallback search) ---

bool PoweredUp::_monitorMatches(const MonitorRequest& monitor, uint16_t ioTypeId) {
  for (uint8_t c = 0; c < monitor.candidateCount; c++) {
    if (monitor.candidateTypes[c] == ioTypeId) {
      return true;
    }
  }
  return false;
}

// Records what the sketch asked for. Calling onTiltChanged() twice for the same port
// replaces the callback rather than filling up the table.
int PoweredUp::_addMonitorRequest(int8_t port, const uint16_t* candidateTypes, uint8_t candidateCount,
                                   uint8_t mode, RawInputHandler callback, const char* label) {
  uint8_t count = candidateCount > MAX_CANDIDATE_TYPES ? MAX_CANDIDATE_TYPES : candidateCount;

  int freeSlot = -1;
  for (uint8_t i = 0; i < MAX_MONITOR_REQUESTS; i++) {
    if (!_monitors[i].inUse) {
      if (freeSlot < 0) {
        freeSlot = i;
      }
      continue;
    }
    if (_monitors[i].port == port && _monitors[i].mode == mode && count > 0 &&
        _monitors[i].candidateCount == count && _monitorMatches(_monitors[i], candidateTypes[0])) {
      _monitors[i].callback = callback;
      _monitors[i].label = label;
      return i;
    }
  }
  if (freeSlot < 0) {
    printf("%s: no room for another subscription (max %d)\n", label, MAX_MONITOR_REQUESTS);
    return -1;
  }

  _monitors[freeSlot].inUse = true;
  _monitors[freeSlot].port = port;
  _monitors[freeSlot].boundPort = -1;
  _monitors[freeSlot].mode = mode;
  _monitors[freeSlot].callback = callback;
  _monitors[freeSlot].candidateCount = count;
  for (uint8_t c = 0; c < count; c++) {
    _monitors[freeSlot].candidateTypes[c] = candidateTypes[c];
  }
  _monitors[freeSlot].label = label;
  return freeSlot;
}

// Picks the subscription that should own this port now that the hub has said what's
// plugged into it, and subscribes to the port's input format for it. A subscription that
// named this port beats one that takes any port, and a port-less one that's already
// driving another port is left where it is.
void PoweredUp::_bindLwp3Port(uint8_t port, uint16_t ioTypeId) {
  int best = -1;
  for (uint8_t i = 0; i < MAX_MONITOR_REQUESTS; i++) {
    const MonitorRequest& m = _monitors[i];
    if (!m.inUse || !_monitorMatches(m, ioTypeId)) {
      continue;
    }
    if (m.port >= 0 && m.port != (int8_t)port) {
      continue; // named a different port - it keeps waiting for that one
    }
    if (m.port < 0 && m.boundPort >= 0 && m.boundPort != (int8_t)port) {
      continue; // "wherever it turns up" already turned up somewhere, and is still there
    }
    if (best < 0 || (_monitors[best].port < 0 && m.port >= 0)) {
      best = i;
    }
  }
  if (best < 0) {
    return; // nothing subscribed to this kind of device
  }

  int idx = _allocSubscription(port);
  if (idx < 0) {
    printf("%s: no room for another port subscription (max %d)\n", _monitors[best].label,
           MAX_SUBSCRIPTIONS);
    return;
  }

  printf("%s: listening on port %d\n", _monitors[best].label, port);
  _subscriptions[idx].mode = _monitors[best].mode;
  _subscriptions[idx].handler = _monitors[best].callback;
  _subscriptions[idx].fromMonitor = true;
  // The subscribe write itself is deferred to handleConnection(): this is reached from
  // inside the notification callback (attach event -> _recordAttached() -> here), and
  // writing to the BLE characteristic from there exhausts the NimBLE stack's buffer pool.
  _subscriptions[idx].reArmPending = true;
  _monitors[best].boundPort = (int8_t)port;
}

// A device left this port, so whatever was listening to it lets go. Subscriptions made
// by monitorInput() stay put - that escape hatch named its port explicitly.
void PoweredUp::_releaseLwp3Port(uint8_t port) {
  for (uint8_t i = 0; i < MAX_MONITOR_REQUESTS; i++) {
    if (_monitors[i].inUse && _monitors[i].boundPort == (int8_t)port) {
      _monitors[i].boundPort = -1;
    }
  }

  int idx = _findSubscription(port);
  if (idx >= 0 && _subscriptions[idx].fromMonitor) {
    _subscriptions[idx] = PortSubscription();
  }
}

void PoweredUp::_monitorWithFallback(int portArg, bool portGiven, const uint16_t* candidateTypes,
                                      uint8_t candidateCount, const char* label, uint8_t mode,
                                      RawInputHandler callback) {
  if (bleProtocol(_slot) != BLE_PROTOCOL_LWP3) {
    printf("%s is only supported for LEGO Powered Up / BOOST / train hubs and remotes\n", label);
    return;
  }

  // A named port is taken literally. This used to fall back to whichever port the device
  // was actually on, which quietly handed this callback a port another subscription was
  // already using - and left the named port with nothing listening to it.
  int8_t wanted = portGiven ? (int8_t)_normalizePort(portArg) : -1;

  int idx = _addMonitorRequest(wanted, candidateTypes, candidateCount, mode, callback, label);
  if (idx < 0) {
    return;
  }

  // Bind it now if the hub has already reported a matching device where we may look.
  for (uint8_t i = 0; i < MAX_ATTACHED_DEVICES; i++) {
    if (!_attached[i].inUse) {
      continue;
    }
    if (wanted >= 0 && _attached[i].port != (uint8_t)wanted) {
      continue;
    }
    if (_monitorMatches(_monitors[idx], _attached[i].ioTypeId)) {
      _bindLwp3Port(_attached[i].port, _attached[i].ioTypeId);
      return;
    }
  }

  // Nothing to go on yet. The subscription stays on file, so the next attach event binds
  // it wherever the device really turns up.
  printf("%s: no matching device attached yet - will start listening automatically once "
         "one does\n", label);
}

// onDistanceChanged()/onTiltChanged() work on both protocols: WeDo 2.0 (via
// _monitorWedoDevice(), which matches against the attach reports the port-type
// characteristic sends - or guesses port A if called before the first one has arrived)
// and LWP3 (via the attached-device directory in _monitorWithFallback()).
//
// The distance reading is 0-10 on both protocols: it's a property of the sensor's UART
// mode, not the hub, so _wedoRangeFormatFor() configures WeDo 2.0's RANGE_10 to match
// LWP3's native DETECT_MODE range instead of the WeDo-specific RANGE_100 used elsewhere.

void PoweredUp::onDistanceChanged(std::function<void(int8_t)> callback) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    _monitorWedoDevice(0, false, ID_DETECT_SENSOR, "onDistanceChanged",
                        [callback](int8_t* v, int size) { if (size >= 1) callback(v[0]); });
    return;
  }
  uint16_t candidates[] = {IO_TYPE_MOTION_SENSOR};
  _monitorWithFallback(0, false, candidates, 1, "onDistanceChanged", DETECT_MODE,
                        [callback](int8_t* v, int size) { if (size >= 1) callback(v[0]); });
}

void PoweredUp::onTiltChanged(std::function<void(int8_t, int8_t)> callback) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    _monitorWedoDevice(0, false, ID_TILT_SENSOR, "onTiltChanged",
                        [callback](int8_t* v, int size) { if (size >= 2) callback(v[0], v[1]); });
    return;
  }
  uint16_t candidates[] = {IO_TYPE_TILT_SENSOR};
  _monitorWithFallback(0, false, candidates, 1, "onTiltChanged", ANGLE_MODE,
                        [callback](int8_t* v, int size) { if (size >= 2) callback(v[0], v[1]); });
}

// --- port(): explicit port targeting + introspection --------------------------------

PortHandle& PoweredUp::port(int portArg) {
  uint8_t normalized = _normalizePort(portArg);
  for (uint8_t i = 0; i < MAX_PORTS; i++) {
    if (_ports[i]._owner != nullptr && _normalizePort(_ports[i]._port) == normalized) {
      return _ports[i];
    }
  }
  for (uint8_t i = 0; i < MAX_PORTS; i++) {
    if (_ports[i]._owner == nullptr) {
      _ports[i]._owner = this;
      _ports[i]._port = portArg;
      return _ports[i];
    }
  }
  printf("port(): no room for another port handle (max %d)\n", MAX_PORTS);
  return _ports[0];
}

uint16_t PoweredUp::_attachedIoType(uint8_t normalizedPort) {
  if (bleProtocol(_slot) == BLE_PROTOCOL_WEDO) {
    return normalizedPort < 2 ? _wedoAttachedDevice[normalizedPort] : 0;
  }
  for (uint8_t i = 0; i < MAX_ATTACHED_DEVICES; i++) {
    if (_attached[i].inUse && _attached[i].port == normalizedPort) {
      return _attached[i].ioTypeId;
    }
  }
  return 0;
}

void PortHandle::onDistanceChanged(std::function<void(int8_t)> callback) {
  if (!_owner) return;
  if (bleProtocol(_owner->_slot) == BLE_PROTOCOL_WEDO) {
    _owner->_monitorWedoDevice(_port, true, ID_DETECT_SENSOR, "port().onDistanceChanged",
                                [callback](int8_t* v, int size) { if (size >= 1) callback(v[0]); });
    return;
  }
  uint16_t candidates[] = {IO_TYPE_MOTION_SENSOR};
  _owner->_monitorWithFallback(_port, true, candidates, 1, "port().onDistanceChanged", DETECT_MODE,
                                [callback](int8_t* v, int size) { if (size >= 1) callback(v[0]); });
}

void PortHandle::onTiltChanged(std::function<void(int8_t, int8_t)> callback) {
  if (!_owner) return;
  if (bleProtocol(_owner->_slot) == BLE_PROTOCOL_WEDO) {
    _owner->_monitorWedoDevice(_port, true, ID_TILT_SENSOR, "port().onTiltChanged",
                                [callback](int8_t* v, int size) { if (size >= 2) callback(v[0], v[1]); });
    return;
  }
  uint16_t candidates[] = {IO_TYPE_TILT_SENSOR};
  _owner->_monitorWithFallback(_port, true, candidates, 1, "port().onTiltChanged", ANGLE_MODE,
                                [callback](int8_t* v, int size) { if (size >= 2) callback(v[0], v[1]); });
}

bool PortHandle::operator==(uint16_t ioType) const {
  if (!_owner) return false;
  return _owner->_attachedIoType(_owner->_normalizePort(_port)) == ioType;
}

// --- remoteButton(): Remote Control up/stop/down, as press/release events -----------

void ButtonEdge::onPressed(std::function<void()> callback, uint16_t repeatMs) {
  _onPressed = callback;
  _repeatMs = repeatMs;
}

void ButtonEdge::onReleased(std::function<void()> callback) {
  _onReleased = callback;
}

RemoteButtonHandle& PoweredUp::remoteButton() {
  return _ensureRemoteButtonGroup(0, false);
}

RemoteButtonHandle& PoweredUp::remoteButton(int portArg) {
  return _ensureRemoteButtonGroup(portArg, true);
}

RemoteButtonHandle& PoweredUp::_ensureRemoteButtonGroup(int portArg, bool portGiven) {
  uint8_t normalized = portGiven ? _normalizePort(portArg) : 0;
  for (uint8_t i = 0; i < MAX_REMOTE_BUTTON_GROUPS; i++) {
    if (_remoteButtons[i]._inUse && _remoteButtons[i]._portGiven == portGiven &&
        (!portGiven || _normalizePort(_remoteButtons[i]._requestedPort) == normalized)) {
      return _remoteButtons[i];
    }
  }
  for (uint8_t i = 0; i < MAX_REMOTE_BUTTON_GROUPS; i++) {
    if (_remoteButtons[i]._inUse) {
      continue;
    }
    _remoteButtons[i]._inUse = true;
    _remoteButtons[i]._portGiven = portGiven;
    _remoteButtons[i]._requestedPort = portGiven ? portArg : 0;

    uint16_t candidates[] = {IO_TYPE_REMOTE_BUTTON};
    RemoteButtonHandle* group = &_remoteButtons[i];
    _monitorWithFallback(portArg, portGiven, candidates, 1, "remoteButton", KEYSD,
                          [this, group](int8_t* value, int size) {
                            this->_handleRemoteButtonRaw(*group, value, size);
                          });
    return _remoteButtons[i];
  }
  printf("remoteButton: no room for another remote button group (max %d)\n", MAX_REMOTE_BUTTON_GROUPS);
  return _remoteButtons[0];
}

// Edge-detection (and Stop-priority) for the remote's up/stop/down level state - the
// library-side replacement for what a sketch used to hand-roll (upHeld/downHeld
// booleans, "up && !upHeld" rising-edge checks).
void PoweredUp::_handleRemoteButtonRaw(RemoteButtonHandle& s, int8_t* value, int size) {
  if (size < 3) {
    return;
  }
  bool up = value[0], stop = value[1], down = value[2];

  if (stop && !s.stop._held) {
    if (s.stop._onPressed) s.stop._onPressed();
  } else if (!stop && s.stop._held) {
    if (s.stop._onReleased) s.stop._onReleased();
  }

  if (!stop) {
    if (up && !s.up._held) {
      if (s.up._onPressed) s.up._onPressed();
      s.up._nextRepeatAt = millis() + s.up._repeatMs;
    } else if (!up && s.up._held) {
      if (s.up._onReleased) s.up._onReleased();
    }
    if (down && !s.down._held) {
      if (s.down._onPressed) s.down._onPressed();
      s.down._nextRepeatAt = millis() + s.down._repeatMs;
    } else if (!down && s.down._held) {
      if (s.down._onReleased) s.down._onReleased();
    }
  } else {
    // Stop takes priority - if up/down were held, treat Stop as also releasing them.
    if (s.up._held && s.up._onReleased) s.up._onReleased();
    if (s.down._held && s.down._onReleased) s.down._onReleased();
  }

  s.up._held = stop ? false : up;
  s.stop._held = stop;
  s.down._held = stop ? false : down;
}

void PoweredUp::stopMonitoring(int port) {
  uint8_t p = _normalizePort(port);

  // WeDo 2.0 side - client-side only, the protocol has no "stop sending" message. The
  // standing subscriptions for this port go too, or the next attach event would rebind
  // the port right back.
  if (p < 2) {
    _wedoDevices[p] = 0;
    _wedoHandlers[p] = nullptr;
    for (uint8_t i = 0; i < MAX_WEDO_SUBSCRIPTIONS; i++) {
      if (_wedoSubscriptions[i].inUse && _wedoSubscriptions[i].port == (int8_t)p) {
        _wedoSubscriptions[i] = WedoSubscription();
      }
    }
  }

  // LWP3 side - the standing subscriptions bound to this port go too, or the next attach
  // event would rebind the port right back.
  int idx = _findSubscription(p);
  if (idx >= 0) {
    if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
      _sendPortInputFormatSetup(p, _subscriptions[idx].mode, false);
    }
    _subscriptions[idx] = PortSubscription();
  }
  for (uint8_t i = 0; i < MAX_MONITOR_REQUESTS; i++) {
    if (_monitors[i].inUse &&
        (_monitors[i].port == (int8_t)p || _monitors[i].boundPort == (int8_t)p)) {
      _monitors[i] = MonitorRequest();
    }
  }

  // Any port() handle for this port becomes stale (its monitoring, if any, just stopped).
  for (uint8_t i = 0; i < MAX_PORTS; i++) {
    if (_ports[i]._owner != nullptr && _normalizePort(_ports[i]._port) == p) {
      _ports[i] = PortHandle();
    }
  }

  // Any remoteButton() group explicitly targeting this port becomes stale too.
  for (uint8_t i = 0; i < MAX_REMOTE_BUTTON_GROUPS; i++) {
    if (_remoteButtons[i]._inUse && _remoteButtons[i]._portGiven &&
        _normalizePort(_remoteButtons[i]._requestedPort) == p) {
      _remoteButtons[i] = RemoteButtonHandle();
    }
  }
}

void PoweredUp::stopMonitoring() {
  _wedoDevices[0] = _wedoDevices[1] = 0;
  _wedoHandlers[0] = _wedoHandlers[1] = nullptr;
  for (uint8_t i = 0; i < MAX_WEDO_SUBSCRIPTIONS; i++) {
    _wedoSubscriptions[i] = WedoSubscription();
  }

  for (uint8_t i = 0; i < MAX_SUBSCRIPTIONS; i++) {
    if (!_subscriptions[i].inUse) {
      continue;
    }
    if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
      _sendPortInputFormatSetup(_subscriptions[i].port, _subscriptions[i].mode, false);
    }
    _subscriptions[i] = PortSubscription();
  }
  for (uint8_t i = 0; i < MAX_MONITOR_REQUESTS; i++) {
    _monitors[i] = MonitorRequest();
  }

  if (_onButtonPressed || _onButtonReleased) {
    _sendHubButtonSubscribe(false);
    _onButtonPressed = nullptr;
    _onButtonReleased = nullptr;
  }

  for (uint8_t i = 0; i < MAX_PORTS; i++) {
    _ports[i] = PortHandle();
  }
  for (uint8_t i = 0; i < MAX_REMOTE_BUTTON_GROUPS; i++) {
    _remoteButtons[i] = RemoteButtonHandle();
  }
}

// ---------------------------------------------------------------------------------------
// Port/Port Mode Information discovery
// ---------------------------------------------------------------------------------------
// Whenever a device attaches (on connect, or later), queries and prints every mode it
// supports as a TSV table, using Port Information Request (0x21) and Port Mode Information
// Request (0x22): https://lego.github.io/lego-ble-wireless-protocol-docs/index.html#port-mode-information-request
// One port is discovered at a time, one mode-info field at a time, all sent from
// handleConnection() (never from the notification callback - see the re-arm comment above).

void PoweredUp::_queuePortDiscovery(uint8_t port, uint16_t ioTypeId) {
  if (_discoveryQueueCount < MAX_DISCOVERY_QUEUE) {
    _discoveryQueue[_discoveryQueueCount] = port;
    _discoveryQueueIoType[_discoveryQueueCount] = ioTypeId;
    _discoveryQueueCount++;
  }
}

void PoweredUp::_sendPortInformationRequest(uint8_t port, uint8_t infoType) {
  uint8_t command[] = {0x05, 0x00, LWP_PORT_INFORMATION_REQUEST, port, infoType};
  bleWriteCommand(_slot, WEDO_OUTPUT, command, sizeof(command));
}

void PoweredUp::_sendPortModeInformationRequest(uint8_t port, uint8_t mode, uint8_t infoType) {
  uint8_t command[] = {0x06, 0x00, LWP_PORT_MODE_INFORMATION_REQUEST, port, mode, infoType};
  bleWriteCommand(_slot, WEDO_OUTPUT, command, sizeof(command));
}

void PoweredUp::_printModeRow() {
  bool isInput = (_discoveryInputModes & (1 << _discoveryMode)) != 0;
  bool isOutput = (_discoveryOutputModes & (1 << _discoveryMode)) != 0;
  static const char* datasetTypeNames[] = {"8-bit", "16-bit", "32-bit", "float"};
  const char* datasetType = (_haveValueFormat && _modeDatasetType < 4) ? datasetTypeNames[_modeDatasetType] : "";

  printf("%d\t0x%04x\t%d\t%s\t%s\t%s\t", _discoveryPort, _discoveryIoTypeId, _discoveryMode,
         _haveName ? _modeName : "", isInput ? "yes" : "no", isOutput ? "yes" : "no");
  if (_haveRaw) printf("%g\t%g\t", _modeRawMin, _modeRawMax); else printf("\t\t");
  if (_havePct) printf("%g\t%g\t", _modePctMin, _modePctMax); else printf("\t\t");
  if (_haveSi) printf("%g\t%g\t", _modeSiMin, _modeSiMax); else printf("\t\t");
  printf("%s\t", _haveSymbol ? _modeSymbol : "");
  if (_haveValueFormat) {
    printf("%d\t%s\t%d\t%d\n", _modeNumDatasets, datasetType, _modeFigures, _modeDecimals);
  } else {
    printf("\t\t\t\n");
  }
}

void PoweredUp::_startModeQuery() {
  _haveName = _haveRaw = _havePct = _haveSi = _haveSymbol = _haveValueFormat = false;
  _discoveryStep = DISCOVERY_STEP_NAME;
  _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_NAME);
  _discoveryStepSentAt = millis();
}

void PoweredUp::_advanceDiscovery() {
  if (_discoveryStep == DISCOVERY_STEP_IDLE) {
    if (_discoveryQueueCount == 0) {
      return;
    }
    _discoveryPort = _discoveryQueue[0];
    _discoveryIoTypeId = _discoveryQueueIoType[0];
    for (uint8_t i = 1; i < _discoveryQueueCount; i++) {
      _discoveryQueue[i - 1] = _discoveryQueue[i];
      _discoveryQueueIoType[i - 1] = _discoveryQueueIoType[i];
    }
    _discoveryQueueCount--;

    printf("Port mode information for port %d (IO type 0x%04x):\n", _discoveryPort, _discoveryIoTypeId);
    printf("Port\tIOType\tMode\tName\tInput\tOutput\tRAW min\tRAW max\tPCT min\tPCT max\tSI min\tSI max\tSymbol\tDatasets\tType\tFigures\tDecimals\n");
    _discoveryStep = DISCOVERY_STEP_PORT_INFO;
    _sendPortInformationRequest(_discoveryPort, PORT_INFO_MODE_INFO);
    _discoveryStepSentAt = millis();
    return;
  }

  bool timedOut = (millis() - _discoveryStepSentAt) > DISCOVERY_STEP_TIMEOUT_MS;
  if (!_discoveryStepComplete && !timedOut) {
    return;
  }
  _discoveryStepComplete = false;

  switch (_discoveryStep) {
    case DISCOVERY_STEP_PORT_INFO:
      if (_discoveryTotalModes == 0) {
        _discoveryStep = DISCOVERY_STEP_IDLE;
        return;
      }
      _discoveryMode = 0;
      _startModeQuery();
      break;
    case DISCOVERY_STEP_NAME:
      _discoveryStep = DISCOVERY_STEP_RAW;
      _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_RAW);
      _discoveryStepSentAt = millis();
      break;
    case DISCOVERY_STEP_RAW:
      _discoveryStep = DISCOVERY_STEP_PCT;
      _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_PCT);
      _discoveryStepSentAt = millis();
      break;
    case DISCOVERY_STEP_PCT:
      _discoveryStep = DISCOVERY_STEP_SI;
      _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_SI);
      _discoveryStepSentAt = millis();
      break;
    case DISCOVERY_STEP_SI:
      _discoveryStep = DISCOVERY_STEP_SYMBOL;
      _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_SYMBOL);
      _discoveryStepSentAt = millis();
      break;
    case DISCOVERY_STEP_SYMBOL:
      _discoveryStep = DISCOVERY_STEP_VALUE_FORMAT;
      _sendPortModeInformationRequest(_discoveryPort, _discoveryMode, MODE_INFO_VALUE_FORMAT);
      _discoveryStepSentAt = millis();
      break;
    case DISCOVERY_STEP_VALUE_FORMAT:
      _printModeRow();
      _discoveryMode++;
      if (_discoveryMode >= _discoveryTotalModes) {
        _discoveryStep = DISCOVERY_STEP_IDLE;
      } else {
        _startModeQuery();
      }
      break;
    default:
      _discoveryStep = DISCOVERY_STEP_IDLE;
  }
}

// ---------------------------------------------------------------------------------------
// Incoming notifications
// ---------------------------------------------------------------------------------------

void PoweredUp::_handleLwp3Notification(uint8_t* data, int size) {
  if (size < 3) {
    return;
  }

  uint8_t messageType = data[2];

  if (messageType == LWP_HUB_PROPERTIES) {
    if (size >= 6 && data[3] == HUB_PROPERTY_BUTTON && data[4] == HUB_PROPERTY_OP_UPDATE) {
      if (data[5] == 1) {
        if (_onButtonPressed) _onButtonPressed();
      } else {
        if (_onButtonReleased) _onButtonReleased();
      }
    }
    return;
  }

  if (messageType == LWP_HUB_ATTACHED_IO) {
    if (size < 5) {
      return;
    }

    uint8_t port = data[3];
    uint8_t event = data[4];

    if (event == 0x01 && size >= 7) {
      // Attached I/O: IO Type ID identifies which sensor/motor is plugged into this port.
      uint16_t ioTypeId = data[5] | (data[6] << 8);
      printf("LWP3 device attached on port %d, IO type ID: 0x%04x\n", port, ioTypeId);

      if (ioTypeId == IO_TYPE_RGB_LIGHT && port != _ledPort) {
        _ledPort = port;
        _ledActiveMode = -1; // force the next write to (re)select its mode on the new port
        _ledModePort = -1;

        // A writeIndexColor()/writeRGB() call made right after connect() - before this
        // attach event arrived - would have gone out on the wrong (default) port and
        // had no visible effect. Since we now know the real port, resend whatever was
        // last requested so it doesn't take a reconnect (or a second explicit call) to
        // show up correctly. Safe to call from here even though this runs inside the
        // notification callback - writeIndexColor()/writeRGB() go through
        // bleWriteCommand(), which queues automatically in that context.
        if (_lastIndexColor >= 0) {
          uint8_t color = (uint8_t)_lastIndexColor;
          _lastIndexColor = -1; // avoid the "unchanged" dedup check skipping the resend
          writeIndexColor(color);
        } else if (_lastRGB[0] >= 0) {
          uint8_t r = (uint8_t)_lastRGB[0], g = (uint8_t)_lastRGB[1], b = (uint8_t)_lastRGB[2];
          _lastRGB[0] = -1;
          writeRGB(r, g, b);
        }
      }

      // Print every mode this port supports as soon as it attaches - every sensor,
      // actuator, internal or external. The actual query writes happen later in
      // handleConnection(), same reason as the re-arm below.
      _queuePortDiscovery(port, ioTypeId);

      // Let onDistanceChanged()/onTiltChanged()/remoteButton() calls waiting on this device resolve now.
      _recordAttached(port, ioTypeId);

      // The hub drops a port's input format subscription whenever its device detaches,
      // so flag it for re-arming if something is still subscribed to this port. Covers
      // monitorInput()'s subscriptions, which _bindLwp3Port() deliberately leaves alone.
      // The actual re-subscribe write happens later in handleConnection(), not here -
      // writing to the BLE characteristic from within this notification callback
      // exhausts the NimBLE stack's buffer pool.
      int idx = _findSubscription(port);
      if (idx >= 0 && _subscriptions[idx].handler != nullptr) {
        _subscriptions[idx].reArmPending = true;
      }
    } else if (event == 0x00) {
      printf("LWP3 device detached from port %d\n", port);

      // Forget what was here, or port() comparisons, _findAttachedPort() and the
      // "already attached?" check in _monitorWithFallback() would all keep answering
      // with a device that's been unplugged.
      for (uint8_t i = 0; i < MAX_ATTACHED_DEVICES; i++) {
        if (_attached[i].inUse && _attached[i].port == port) {
          _attached[i] = AttachedDeviceInfo();
        }
      }
      _releaseLwp3Port(port);
    }
    return;
  }

  if (messageType == LWP_PORT_VALUE_SINGLE) {
    if (size < 5) {
      return;
    }

    uint8_t port = data[3];
    int idx = _findSubscription(port);
    if (idx < 0 || _subscriptions[idx].handler == nullptr) {
      return;
    }

    int valueSize = size - 4;
    int8_t values[valueSize];
    for (int i = 0; i < valueSize; i++) {
      values[i] = (int8_t)data[4 + i];
    }
    _subscriptions[idx].handler(values, valueSize);
    return;
  }

  if (messageType == LWP_PORT_INFORMATION) {
    if (size < 6) {
      return;
    }
    uint8_t port = data[3];
    uint8_t infoType = data[4];
    if (infoType == PORT_INFO_MODE_INFO && port == _discoveryPort &&
        _discoveryStep == DISCOVERY_STEP_PORT_INFO && size >= 11) {
      _discoveryTotalModes = data[6];
      _discoveryInputModes = data[7] | (data[8] << 8);
      _discoveryOutputModes = data[9] | (data[10] << 8);
      _discoveryStepComplete = true;
    }
    return;
  }

  if (messageType == LWP_PORT_MODE_INFORMATION) {
    if (size < 6) {
      return;
    }
    uint8_t port = data[3];
    uint8_t mode = data[4];
    uint8_t infoType = data[5];
    if (port != _discoveryPort || mode != _discoveryMode) {
      return; // not the response we're currently waiting for
    }

    if (infoType == MODE_INFO_NAME && _discoveryStep == DISCOVERY_STEP_NAME) {
      int len = size - 6;
      if (len > 11) len = 11;
      if (len < 0) len = 0;
      memcpy(_modeName, data + 6, len);
      _modeName[len] = 0;
      _haveName = true;
      _discoveryStepComplete = true;
    } else if (infoType == MODE_INFO_RAW && _discoveryStep == DISCOVERY_STEP_RAW && size >= 14) {
      memcpy(&_modeRawMin, data + 6, 4);
      memcpy(&_modeRawMax, data + 10, 4);
      _haveRaw = true;
      _discoveryStepComplete = true;
    } else if (infoType == MODE_INFO_PCT && _discoveryStep == DISCOVERY_STEP_PCT && size >= 14) {
      memcpy(&_modePctMin, data + 6, 4);
      memcpy(&_modePctMax, data + 10, 4);
      _havePct = true;
      _discoveryStepComplete = true;
    } else if (infoType == MODE_INFO_SI && _discoveryStep == DISCOVERY_STEP_SI && size >= 14) {
      memcpy(&_modeSiMin, data + 6, 4);
      memcpy(&_modeSiMax, data + 10, 4);
      _haveSi = true;
      _discoveryStepComplete = true;
    } else if (infoType == MODE_INFO_SYMBOL && _discoveryStep == DISCOVERY_STEP_SYMBOL) {
      int len = size - 6;
      if (len > 5) len = 5;
      if (len < 0) len = 0;
      memcpy(_modeSymbol, data + 6, len);
      _modeSymbol[len] = 0;
      _haveSymbol = true;
      _discoveryStepComplete = true;
    } else if (infoType == MODE_INFO_VALUE_FORMAT && _discoveryStep == DISCOVERY_STEP_VALUE_FORMAT && size >= 10) {
      _modeNumDatasets = data[6];
      _modeDatasetType = data[7];
      _modeFigures = data[8];
      _modeDecimals = data[9];
      _haveValueFormat = true;
      _discoveryStepComplete = true;
    }
  }
}

void PoweredUp::_handleWedoNotification(uint8_t* data, int size) {
  
  printf("Received WeDo notification, size: %d\nData:", size);
  for (int i = 0; i < size; i++) {
    printf(" %02x", data[i]);
  }
  printf("\n");
  if (size < 3) {
    printf("Invalid data size: %d\n", size);
    return;
  }

  uint8_t port = data[1];

  if (port > 0 && port <= 2) {
    uint8_t idx = port - 1;
    if (_wedoDevices[idx] > 0 && _wedoHandlers[idx]) {
      if (_wedoDevices[idx] == ID_DETECT_SENSOR) {
        // send only one value (0-100)
        int8_t callback_data[] = {(int8_t)data[2]};
        _wedoHandlers[idx](callback_data, 1);
      } else if (_wedoDevices[idx] == ID_TILT_SENSOR) {
        // send two values (-45 -> 45)
        if (size >= 4) {
          int8_t callback_data[] = {(int8_t)data[2], (int8_t)data[3]};
          _wedoHandlers[idx](callback_data, 2);
        }
      }
    } else {
      printf("Can't handle the message - perhaps you didn't use onDistanceChanged()/onTiltChanged()\n");
    }
  }
}

// WeDo 2.0 port type characteristic (0x1527) notifications - what's plugged into which
// external port. Confirmed live on real hardware (attach/detach events logged while
// physically plugging/unplugging a motor and a motion/tilt sensor on both ports):
//   Attach (12 bytes): data[0]=port (1-based, 1 or 2), data[1]=0x01 (attached),
//     data[2]=port again (0-based, 0 or 1), data[3]=IO Type ID (0x01=motor,
//     0x22=tilt, 0x23=motion, matching ID_MOTOR/ID_TILT_SENSOR/ID_DETECT_SENSOR),
//     data[4..11]=default/current mode settings (unused here).
//   Detach (2 bytes): data[0]=port (1-based), data[1]=0x00.
void PoweredUp::_handleWedoPortTypeNotification(uint8_t* data, int size) {
  if (size < 2) {
    return;
  }

  uint8_t port = data[0];
  if (port < 1 || port > 2) {
    return;
  }
  uint8_t idx = port - 1;

  bool attached = data[1] != 0;
  if (!attached) {
    printf("WeDo device detached from port %d\n", port);
    _wedoAttachedDevice[idx] = 0;
    for (uint8_t i = 0; i < MAX_WEDO_SUBSCRIPTIONS; i++) {
      if (_wedoSubscriptions[i].inUse && _wedoSubscriptions[i].boundPort == (int8_t)idx) {
        _wedoSubscriptions[i].boundPort = -1;
      }
    }
    // The port's configuration belongs to the device that just left. Standing
    // subscriptions live in _wedoSubscriptions, so whatever gets plugged in next is
    // bound from there rather than from whatever happened to be here before.
    _wedoDevices[idx] = 0;
    _wedoHandlers[idx] = nullptr;
    return;
  }

  uint8_t deviceType = size >= 4 ? data[3] : 0;
  printf("WeDo device attached on port %d, device type: %d\n", port, deviceType);
  _wedoAttachedDevice[idx] = deviceType;

  // Rebind from scratch on every attach. Clearing first means a port that was
  // configured for something else can't keep decoding this device's readings under the
  // old device's format (a tilt sensor reporting distances, say), and rebinding means a
  // sensor can be unplugged, swapped and replugged any number of times - the port
  // definition doesn't survive an unplug, so it has to be resent regardless.
  _wedoDevices[idx] = 0;
  _wedoHandlers[idx] = nullptr;
  _bindWedoPort(idx);
}

void PoweredUp::_handleNotification(uint8_t* data, int size, BLENotificationSource source) {
  if (_userNotificationOverride != nullptr) {
    _userNotificationOverride(data, size);
    return;
  }

  if (bleProtocol(_slot) == BLE_PROTOCOL_LWP3) {
    _handleLwp3Notification(data, size);
    return;
  }

  if (bleProtocol(_slot) != BLE_PROTOCOL_WEDO) {
    return;
  }

  if (source == BLE_NOTIFY_PORT_TYPE) {
    _handleWedoPortTypeNotification(data, size);
    return;
  }

  if (source == BLE_NOTIFY_WEDO_BUTTON) {
    // Single byte: 0x00/0x01 (released/pressed) - same shape the LWP3 side dispatches to.
    if (size >= 1) {
      if (data[0] == 1) {
        if (_onButtonPressed) _onButtonPressed();
      } else {
        if (_onButtonReleased) _onButtonReleased();
      }
    }
    return;
  }

  _handleWedoNotification(data, size);
}

void PoweredUp::addNotificationHandler(std::function<void(uint8_t*, int)> f) {
  // Overrule the standard notification handling
  _userNotificationOverride = f;
}
