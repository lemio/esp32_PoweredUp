You can flash the examples below directly to an esp32-s3-devkit-c1 or a compatible esp32-s3 board using the [Web flasher](https://lemio.github.io/esp32_PoweredUp/)

# Powered Up BLE library for ESP32
A library to control LEGO Powered Up, BOOST, train, Duplo, and WeDo 2.0 hubs (plus the
Powered Up Remote Control) from an ESP32 over Bluetooth low energy.

## button_motor.ino — a physical remote using the ESP32's own button

The simplest possible example: hold the ESP32's built-in button (pin 0) and whichever
motor is plugged into the hub runs, with the hub's LED turning green while held and red
when released. A good starting point if you want to build your own physical remote for
a LEGO creation.

```cpp
#include <PoweredUp.h>

PoweredUp myHub(nullptr, DEVICE_TYPE_ANY_HUB);

void setup() {
  myHub.connect();
}

void loop() {
  myHub.handleConnection();
  bool pressed = !digitalRead(0); // built-in button is active-low
  myHub.writeIndexColor(pressed ? LEGO_COLOR_GREEN : LEGO_COLOR_RED);
  myHub.writeMotor(pressed * 100);
}
```

https://github.com/user-attachments/assets/e2af3bfe-5ca0-48af-92df-ad1df9a39a3a

See [`examples/button_motor`](examples/button_motor) for the full sketch.

## sensor_motor.ino — a motor and LED driven by a distance sensor

A motor and a WeDo 2.0 detect (infrared distance) sensor plugged into the same hub, in
either port - the sensor's reading drives both the motor's speed and the hub's LED
colour, so moving your hand closer/further changes both at once. `onDistanceChanged()`
with no port given finds the sensor itself, wherever it's plugged in.

```cpp
#include <PoweredUp.h>

PoweredUp myHub(nullptr, DEVICE_TYPE_ANY_HUB);
int detectSensorValue = 0;

void setup() {
  myHub.connect();
  myHub.onDistanceChanged([](int8_t distance){ detectSensorValue = distance; });
}

void loop() {
  myHub.handleConnection();
  myHub.writeIndexColor(detectSensorValue);            // 0-10, matches the LEGO_COLOR_* index range directly
  myHub.writeMotor(100 - detectSensorValue * 10);       // scaled up to the full -100..100 motor range
}
```

https://github.com/user-attachments/assets/2bf333ff-1557-4f30-bbb0-2b545ad304cf

See [`examples/sensor_motor`](examples/sensor_motor) for the full sketch.

## train_hub.ino — a standalone train hub demo, no remote needed

Drives whichever motor is plugged into a Powered Up train hub back and forth, lighting
the hub's LED green while going forward and red while going backward. If a tilt sensor
is plugged in too, its x/y angle is printed to Serial.

```cpp
#include <PoweredUp.h>

PoweredUp hub(nullptr, DEVICE_TYPE_POWERED_UP_HUB);

void setup() {
  Serial.begin(115200);
  hub.connect();
  hub.onTiltChanged([](int8_t x, int8_t y){
    Serial.printf("Tilt angle: x=%d y=%d\n", x, y);
  });
}

void loop() {
  hub.handleConnection();
  hub.writeRGB(0, 255, 0); hub.writeMotor(60);  delay(2000); // forward, green
  hub.writeRGB(255, 0, 0); hub.writeMotor(-60); delay(2000); // backward, red
}
```


https://github.com/user-attachments/assets/4fce5972-02e1-4a69-b312-a5ac862f64a9

See [`examples/train_hub`](examples/train_hub) for the full sketch.

## multi_sensor.ino — several sensors on one hub, each on its own port

Subscribes to more than one sensor on a single hub at the same time and lets the library
work out which port each one is on. A named port is taken literally, so
`hub.port('A')` only ever binds to port A. A port-less call like `onDistanceChanged()`
takes whichever free port the sensor turns up on, and corrects itself once the hub
reports the attachment. Nothing ever reconfigures a port that already holds a different
device, so a tilt sensor keeps reporting angles even if a distance subscription is
registered afterwards.

```cpp
#include <PoweredUp.h>

PoweredUp hub(nullptr);

void setup() {
  Serial.begin(115200);
  hub.connect();
  hub.port('A').onTiltChanged([](int8_t x, int8_t y){
    Serial.printf("Tilt angle A: x=%d y=%d\n", x, y);
  });
  hub.port('B').onTiltChanged([](int8_t x, int8_t y){
    Serial.printf("Tilt angle B: x=%d y=%d\n", x, y);
  });
  hub.onDistanceChanged([](int8_t distance){
    Serial.printf("Distance: %d\n", distance);
  });
}

void loop() {
  hub.handleConnection();
}
```

Plug and unplug sensors while it runs - the Serial log shows each attach event and which
port every subscription ends up on.

See [`examples/multi_sensor`](examples/multi_sensor) for the full sketch.

## train_remote.ino — a Remote Control driving a train hub, with a speed gauge

A LEGO Powered Up Remote Control drives a real train hub: the remote's up/down buttons
step the speed by 10% and repeat while held (like a keyboard key), the stop button halts
immediately and takes priority over up/down, and the hub's own button cycles through the
built-in LEGO colours on both the hub's and remote's LEDs at once. A 5-pixel NeoPixel
strip mirrors the current speed and colour as a lit bar.

```cpp
#include <PoweredUp.h>

PoweredUp hub(nullptr, DEVICE_TYPE_ANY_HUB);
PoweredUp remote(nullptr, DEVICE_TYPE_POWERED_UP_REMOTE);
int trainSpeed = 0;

void setup() {
  hub.connect();
  remote.connect();

  RemoteButtonHandle& btn = remote.remoteButton('A');
  btn.up.onPressed([](){ trainSpeed += 10; hub.writeMotor(trainSpeed); }, 200); // repeats every 200ms while held
  btn.down.onPressed([](){ trainSpeed -= 10; hub.writeMotor(trainSpeed); }, 200);
  btn.stop.onPressed([](){ trainSpeed = 0; hub.writeMotor(0); });

  hub.onButtonPressed([](){ /* cycle colourIndex, write it to both hub and remote */ });
}

void loop() {
  hub.handleConnection();
  remote.handleConnection();
}
```


https://github.com/user-attachments/assets/c71aea2d-3c68-4cb2-ad71-37646fdeaefe

See [`examples/train_remote`](examples/train_remote) for the full sketch (colour gauge, direction light, and NeoPixel wiring notes included).

## wifi_control.ino — driving a motor from a web page instead of BLE input

No physical controls at all - the ESP32 joins your WiFi network and serves a page styled
like the physical LEGO Powered Up Remote Control: a grey "+" on top drives the motor
forward, a red STOP in the middle stops it, a grey "-" on the bottom drives it backward.
Big, centred buttons that highlight while pressed (mouse or touch), firing each command
in the background with `fetch()` so the page never reloads. Also starts an mDNS
responder, so the page is reachable at `http://esp-poweredup.local/` out of the box -
even flashed straight from `pio run --target upload` with no other setup - instead of
having to look up its IP address.

The WiFi SSID/password are fixed-size placeholder strings (`|*S*|`, `|*P*|`) rather than
plain literals - there's no sensible default for "your WiFi," so if you're flashing a
prebuilt binary via a browser tool like [ESP32-S3-Flasher](https://github.com/lemio/ESP32-S3-Flasher),
these get patched with your real values at flash time, no recompiling needed. Building
from source yourself instead? Just replace the placeholder strings directly. The mDNS
hostname works differently: its compiled-in default is already a real, usable value
(`esp-poweredup`) - the flasher can still search-and-replace it for a different name if
you're running more than one board on the same network, but you don't have to.

```cpp
#include <PoweredUp.h>
#include <WiFi.h>
#include <ESPmDNS.h>

PoweredUp hub(nullptr, DEVICE_TYPE_ANY_HUB);
WiFiServer server(80);

// WIFI_SSID/WIFI_PASSWORD: patched at flash time by a browser flashing tool, or replace
// directly if building from source - no sensible default for "your WiFi" exists.
const char WIFI_SSID[100]     = "|*S*|";
const char WIFI_PASSWORD[100] = "|*P*|";
// MDNS_HOST: a real, usable default - works even without ever touching a flashing tool.
const char MDNS_HOST[100]     = "esp-poweredup";

void setup() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  MDNS.begin(MDNS_HOST);            // reachable at http://esp-poweredup.local/
  MDNS.addService("http", "tcp", 80);
  server.begin();
  hub.connect();
}

void loop() {
  hub.handleConnection();
  // serves the remote-control page at "/", and GET /F, /S, /B -> hub.writeMotor(1, 100/0/-100)
}
```

<!-- TODO: record and embed a demo video here, showing the new remote-control-style buttons
     (grey +/- and red STOP) and the motor responding to each press. The previous demo
     video (user-attachments/assets/40f05ed4-...) showed the old link-based UI, since
     replaced by the button UI above - needs a fresh recording, not the old link. -->

See [`examples/wifi_control`](examples/wifi_control) for the full sketch, or flash it
straight from your browser (no PlatformIO needed) - see [Flashing prebuilt firmware](#flashing-prebuilt-firmware) below.

## analog_throttle.ino — a breadboard knob driving a real LEGO train

A potentiometer wired to the ESP32 acts as a physical throttle for a real LEGO train:
turn it one way to speed up forward, the other way to reverse, center to stop. Pressing
the hub's own button cycles its LED colour. A phone running the official LEGO app, or a
JavaScript library on a laptop, can't do the knob part at all - they have no GPIO/ADC to
read a physical control from. An ESP32 does, and this library turns that knob into a
motor command in about 10 lines of code:

```cpp
#include <PoweredUp.h>

#define POT_PIN 8  // potentiometer wiper - outer legs to 3.3V and GND

// DEVICE_TYPE_ANY_HUB connects to any supported LEGO hub - WeDo 2.0, Powered Up, BOOST,
// train, Duplo - but never a Remote Control, so this can't accidentally connect to a
// nearby remote instead of an actual hub.
PoweredUp hub(nullptr, DEVICE_TYPE_ANY_HUB);
int colorIndex = 1;

void setup() {
  hub.connect();
  hub.onButtonPressed([](){
    colorIndex = (colorIndex % 9) + 1;
    hub.writeIndexColor(colorIndex); // pressing the hub's own button cycles its LED
  });
}

void loop() {
  hub.handleConnection();
  int raw = analogRead(POT_PIN);
  hub.writeMotor(map(raw, 0, 4095, -100, 100)); // knob position -> motor speed/direction
  delay(20);
}
```

https://github.com/user-attachments/assets/2882283a-fe08-4eb5-a36b-bcaa7341feb5

See [`examples/analog_throttle`](examples/analog_throttle) for the full sketch and wiring notes.

## Flashing prebuilt firmware

Don't want to install PlatformIO just to try an example? Every example above is also
built automatically and published as ready-to-flash firmware at
**[lemio.github.io/esp32_PoweredUp](https://lemio.github.io/esp32_PoweredUp/)** - open it
in Chrome or Edge (Web Serial support required), pick an example, plug in your ESP32
over USB, and flash it directly from the browser using
[ESP32-S3-Flasher](https://github.com/lemio/ESP32-S3-Flasher). `wifi_control.ino`'s page
lets you fill in your WiFi SSID/password/mDNS hostname there too - they get patched into
the firmware at flash time, no recompiling needed.

## Version 3.0.0 - the `PoweredUp` class 🎉

This library was rewritten around a new `PoweredUp` class that replaces the old `Wedo`
class - **this is a breaking change**, not a drop-in upgrade. Make one `PoweredUp`
object per LEGO device you want to talk to (a hub, a Remote Control, ...) - you can have
more than one connected to the same ESP32 at the same time. The same methods
(`writeMotor()`, `writeIndexColor()`, `onDistanceChanged()`, ...) work the same way
whether you're talking to a WeDo 2.0 hub or a Powered Up / BOOST / train hub - the
library figures out which protocol it's using for you. It's still built on the modern
**NimBLE-Arduino** library for performance, stability, and low memory usage.

### Requirements

- **ESP32** board
- **NimBLE-Arduino** library (automatically installed via Arduino Library Manager or add to platformio.ini)

### Installation

#### Arduino IDE
1. Install NimBLE-Arduino from Library Manager (search for "NimBLE-Arduino" by h2zero)
2. Install this library from Library Manager or download from GitHub

#### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    h2zero/NimBLE-Arduino
    lemio/esp32_PoweredUp
```

## API Reference

### PoweredUp(const char\* name = nullptr, LegoDeviceType deviceType = DEVICE_TYPE_ANY)

Makes an object representing one LEGO device. Pass a `name` to match a specific
device's advertised name, a `deviceType` to match a specific kind of device (see below),
both, or neither to connect to the first supported LEGO device found.

```cpp
PoweredUp hub;                                             // any supported LEGO device
PoweredUp hub(nullptr, DEVICE_TYPE_ANY_HUB);                // any hub, but never a Remote Control
PoweredUp remote(nullptr, DEVICE_TYPE_POWERED_UP_REMOTE);   // specifically a Remote Control
PoweredUp hub("My Train", DEVICE_TYPE_POWERED_UP_HUB);      // by name and kind
```

`LegoDeviceType` values: `DEVICE_TYPE_ANY`, `DEVICE_TYPE_ANY_HUB`, `DEVICE_TYPE_WEDO_HUB`,
`DEVICE_TYPE_DUPLO_TRAIN`, `DEVICE_TYPE_BOOST_HUB`, `DEVICE_TYPE_POWERED_UP_HUB` (train
hub, city hub, etc.), `DEVICE_TYPE_POWERED_UP_REMOTE`.

### connect(uint32_t timeoutMs = 30000)

Starts looking for the device and blocks until it's connected (or `timeoutMs` has
passed). Safe to call on more than one `PoweredUp` object in a row in `setup()`.

### connected() / ready()

`connected()` is true once the BLE connection is up. `ready()` is true once the last
command you sent has finished.

### handleConnection()

Call this every `loop()` - keeps the connection alive and lets background work
(reconnecting, re-subscribing sensors, discovering what's plugged in) happen.

### Actuators

```cpp
void writeMotor(int speed);                 // drives whichever motor is plugged in
void writeMotor(int port, int speed);       // port: 'A'/'B' or 1/2, if you need to be specific
void writeIndexColor(uint8_t color);        // a built-in LEGO colour, 0-10 - see LEGO_COLOR_* below
void writeRGB(uint8_t red, uint8_t green, uint8_t blue); // mix your own colour, each channel 0-255
void writeSound(unsigned int frequency, unsigned int length); // WeDo hubs only - piezo speaker
void writeLight(int value);                 // a plugged-in simple LEGO Power Functions light, -100..100
```

`speed`/`value` ranges are all -100 to 100. `writeMotor(speed)`/`writeLight(value)` find
the right port themselves - you only need the port-specific overload if more than one
matching device is attached (e.g. two motors on one hub).

`writeIndexColor()`'s built-in colours:
<pre>
#define LEGO_COLOR_BLACK 0
#define LEGO_COLOR_PINK 1
#define LEGO_COLOR_PURPLE 2
#define LEGO_COLOR_BLUE 3
#define LEGO_COLOR_CYAN 4
#define LEGO_COLOR_LIGHTGREEN 5
#define LEGO_COLOR_GREEN 6
#define LEGO_COLOR_YELLOW 7
#define LEGO_COLOR_ORANGE 8
#define LEGO_COLOR_RED 9
#define LEGO_COLOR_WHITE 10
</pre>

### Buttons & sensors

Callbacks are lambdas, not raw byte arrays - `[](){ ... }` for a button,
`[](int8_t x, int8_t y){ ... }` for a tilt sensor, etc. Since they're `std::function`,
they can capture variables (`[&colorIndex](){ ... }`), unlike a plain C function pointer.

```cpp
void onButtonPressed(std::function<void()> callback);   // the hub's own physical button - no port, there's only one
void onButtonReleased(std::function<void()> callback);

void onDistanceChanged(std::function<void(int8_t distance)> callback);        // a distance/proximity sensor, auto-detected, 0-10
void onTiltChanged(std::function<void(int8_t x, int8_t y)> callback);         // tilt angle, x/y degrees, -45 to 45, auto-detected
```

Both work on WeDo 2.0 hubs and Powered Up / BOOST / train hubs, and find the right
device themselves. A WeDo 2.0 hub does report what's plugged into each port (via its
port-type characteristic, the same way it reports attach/detach events), but if you
call `onDistanceChanged()`/`onTiltChanged()` before that report has arrived - e.g.
right after `connect()` - there's nothing to match against yet, so it assumes port A
and corrects itself automatically once the real attach event comes in.

For explicit port targeting instead of auto-detection, or to ask what's actually plugged
into a port, use `port('A')` (or `1`, `2`, ...):

```cpp
hub.port('A').onDistanceChanged([](int8_t distance){ ... });
hub.port('A').onTiltChanged([](int8_t x, int8_t y){ ... });

if (hub.port('A') == IO_TYPE_MOTION_SENSOR) {
  Serial.println("There's a motion sensor on port A");
}
```

`port()`'s comparison checks the same attach-tracking the library already uses
internally, so it's most reliable once `handleConnection()` has run for a bit after
`connect()` - checked immediately in `setup()`, a device that's physically plugged in
may not have reported its attach event yet.

A Remote Control's three buttons (up/stop/down) are addressed the same way, one handle
per port covering all three:

```cpp
RemoteButtonHandle& btn = remote.remoteButton('A'); // or remote.remoteButton() to auto-detect
btn.up.onPressed([](){ ... });
btn.up.onReleased([](){ ... });
btn.down.onPressed([](){ ... }, 200); // optional repeatMs - keep firing every 200ms while held, like a keyboard key
btn.stop.onPressed([](){ ... });      // stop takes priority: while held, up/down presses are suppressed
```

Advanced: `monitorInput(int port, RawInputHandler callback, uint8_t mode)` listens to a
specific mode yourself, callback shaped `std::function<void(int8_t* value, int size)>` -
see the `*Mode` enums in `PoweredUp.h`, or the "Port mode information" table the library
prints to Serial whenever something attaches, for what modes a given device supports.
Most people should use the methods above instead.

```cpp
void stopMonitoring(int port);   // stop listening on one port (including port()/remoteButton() handles targeting it)
void stopMonitoring();           // stop all monitoring on this connection
```

**A lambda footgun to know about:** a callback that captures a local variable *by
reference* will read freed memory if that local goes out of scope before the callback
fires later - general to `std::function`/lambdas, not specific to this library. Capture
by value, or use globals/member variables for anything a callback needs to persist.

### Advanced / escape hatches

```cpp
void writeCommand(uint8_t* command, int size, int type = WEDO_OUTPUT); // send a raw command yourself
void writePortDefinition(uint8_t port, uint8_t type, uint8_t mode, uint8_t format); // WeDo 2.0 only
void addNotificationHandler(std::function<void(uint8_t*, int)> f); // replaces the library's own notification handling with your own
```

Most sketches won't need these - `writeCommand()`'s `type` defaults to `WEDO_OUTPUT`
(motor/LED/sound commands); `WEDO_INPUT` is for the handful of WeDo-specific commands
that need it.

## Examples

See each example's own section above for a description, code snippet, and demo video:
[button_motor.ino](#button_motorino--a-physical-remote-using-the-esp32s-own-button),
[sensor_motor.ino](#sensor_motorino--a-motor-and-led-driven-by-a-distance-sensor),
[train_hub.ino](#train_hubino--a-standalone-train-hub-demo-no-remote-needed),
[multi_sensor.ino](#multi_sensorino--several-sensors-on-one-hub-each-on-its-own-port),
[train_remote.ino](#train_remoteino--a-remote-control-driving-a-train-hub-with-a-speed-gauge),
[wifi_control.ino](#wifi_controlino--driving-a-motor-from-a-web-page-instead-of-ble-input),
[analog_throttle.ino](#analog_throttleino--a-breadboard-knob-driving-a-real-lego-train).

## Prior art

* [Official WEDO2.0 SDK from LEGO ](https://education.lego.com/en-us/support/wedo-2/developer-kits)
* [Nodejs implementation for Wedo2.0](https://github.com/vheun/wedo2)
* [Controlling a motor in linux with USB dongle](http://www.ev3dev.org/docs/tutorials/controlling-wedo2-motor/)
* [Sniffing data from WEDO2.0](http://ofalcao.pt/blog/2016/wedo-2-0-reverse-engineering)
* [Controlling a motor ](http://ofalcao.pt/blog/2016/controlling-wedo-2-0-motor-from-linux)
* [Controlling RGB LED in python](http://ofalcao.pt/blog/2016/wedo-2-0-colors-with-python)
* [Making apps for WEDO2.0 in app-inventor](http://ofalcao.pt/blog/2016/lego-wedo-2-0-with-mit-app-inventor)
* [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) - The BLE library powering this project
* [nRF Connect App](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile)
