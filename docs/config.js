// ESP32 Flash Configuration
// This file contains the configuration for both index.html and wizard.html

const CONFIG = {
    DISCONNECT_WAIT_MS: 1500,
    // Used for the whole session - the initial sync AND the flash write itself (the
    // ESP32 ROM bootloader auto-detects baud from the sync command's own byte pattern,
    // so there's no separate low-speed handshake stage to worry about). 115200 is the
    // usual "safe" default, but it is NOT universally the most reliable one: on a
    // classic ESP32 WROOM board with a CH340 USB-serial chip, 115200 reproducibly
    // corrupted the sync/flash over Web Serial (garbled reads, repeated resets) across
    // 8 attempts on 2 different cables/ports, while 460800 flashed cleanly and
    // repeatedly (and ~4x faster) - a likely CH340 baud-divisor quirk, not a cable/port
    // issue. If flashing is unreliable on your hardware, don't assume slower is safer -
    // try a higher rate (e.g. 460800 or 921600) as well as a lower one.
    BAUD_RATE: 460800,
    // Cosmetic only (titles, button/log text, the generated esptool.py comment) - it
    // does NOT restrict which chip can connect. esptool-js's own sync/handshake
    // auto-detects the real connected chip (plain ESP32, S2, S3, C3, C6, ...)
    // regardless of this value, so the whole ESP32 family already flashes fine as-is.
    // Set this to whatever your project actually targets, e.g. "ESP32" or "ESP32-C3".
    CHIP_NAME: "ESP32",
    // Which USB-to-serial bridge chips show up in the browser's "Connect" device
    // picker. A device not covered by any entry here simply won't be selectable -
    // add its usbVendorId (and optionally usbProductId) below.
    FILTERS: [
        {usbVendorId: 0x1A86},    // WCH (CH340/CH343/CH9102) - most common on generic/plain ESP32 dev boards
        {usbVendorId: 0x10C4},    // SILICON_LABS (CP210x) - common on ESP32-S3-DevKitC and similar
        {usbVendorId: 0x303A},    // ESPRESSIF (native USB-JTAG/CDC, e.g. ESP32-S3/C3/C6 native USB port)
        {usbVendorId: 0x0403},    // FTDI
        {usbVendorId: 0x1B4F},    // SparkFun
        {usbVendorId: 0x2341}     // Arduino
    ]
    // Vendors IDs are protected by USB-IF and can be found online
    // https://devicehunt.com/view/type/usb/vendor/10C4/product/EA60
    //                              {usbProductId: 0xEA60,  usbVendorId: 0x10C4}
    // ESP32-S3-Devkit-C1 UART Port: {(CP210x UART Bridge),  (SILICON_LABS)}
    // ESP32-S3-Devkit-C1 USB Port: {usbProductId: 4097 0x1001, usbVendorId: 12346 = 0x303A (ESPRESSIF)}
    //
    // A consuming repo can also override FILTERS (and CHIP_NAME) without touching this
    // file at all, via flasher-manifest.yml's top-level `site:` block - see README.md's
    // "Configure it for your own project" section. Shape:
    //   site:
    //     chipName: ESP32
    //     filters:
    //       - usbVendorId: 0x1A86
};

// No firmwares are hardcoded here anymore - this file is a generic flashing UI that any
// repo can point at its own manifest.json (see README.md's "For PlatformIO projects"
// section), or you can hand-add entries here directly. Shape of an entry, for reference:
//
// const FIRMWARE_CONFIGS = {
//   'my-firmware-key': {
//     name: 'Human-readable name',
//     description: 'Brief description (wizard only)',
//     hardware: 'ESP32-S3-DevKitC-1',      // board/chip this firmware targets - shown in the UI
//     expectedBehavior: [                   // wizard only
//       'What happens after flashing',
//       'Can include HTML like <b>bold</b> or <a href="...">links</a>'
//     ],
//     files: [                              // firmware files to flash, standard ESP32 layout
//       { path: 'path/to/bootloader.bin', offset: 0x0000 },
//       { path: 'path/to/partitions.bin', offset: 0x8000 },
//       { path: 'path/to/boot_app0.bin',  offset: 0xe000 },
//       { path: 'path/to/firmware.bin',   offset: 0x10000 }
//     ],
//     variables: [                          // optional: flash-time-patchable placeholders
//       {
//         firmware_name: '|*S*|',           // exact string reserved in the compiled binary
//         readable_name: 'WiFi Name',       // label shown to the user
//         default_value: 'MyNetwork',       // shown as the default in the UI
//         max_length: 100,                  // must match the fixed-size array in firmware source
//         postfix: '.local'                 // optional: appended after the input in the UI
//       },
//       {                                   // type: 'enum' renders a <select> dropdown instead
//         firmware_name: '|*ROTATION*|',    // of a free-text <input> - same patching mechanism,
//         readable_name: 'Screen Rotation', // just a constrained set of choices. `value` is what
//         type: 'enum',                     // gets written into the firmware; `label` is what the
//         options: [                        // user sees. A plain string entry (no {value,label})
//           { value: '0', label: '0°' },    // is used as both.
//           { value: '1', label: '90°' },
//           { value: '2', label: '180°' },
//           { value: '3', label: '270°' }
//         ],
//         default_value: '1',
//         max_length: 16                    // still needs room for firmware_name itself (here,
//       },                                  // '|*ROTATION*|' is 12 bytes) plus a null terminator
//       {                                   // type: 'timezone' renders a <select> populated from
//         firmware_name: '|*TZ*|',          // the browser's own Intl.supportedValuesOf('timeZone')
//         readable_name: 'Timezone',        // (every IANA zone, e.g. "America/New_York"), defaulting
//         type: 'timezone',                 // to Intl.DateTimeFormat().resolvedOptions().timeZone -
//         max_length: 64                    // the browser's own detected zone - unless a prior
//       }                                   // choice is already in localStorage. Firmware doesn't
//     ]                                     // get the IANA name though: right before patching, it's
//                                            // looked up in posix_tz_db.js and the POSIX TZ string
//                                            // (e.g. "EST5EDT,M3.2.0,M11.1.0") is written instead -
//                                            // what configTzTime()-style APIs actually need. Give
//                                            // max_length enough room for the longest POSIX strings
//                                            // (~44 bytes) plus the placeholder itself and a null
//                                            // terminator - 64 is a safe default.
//   }
// };
const FIRMWARE_CONFIGS = {};

// Export for ES6 modules
export { CONFIG, FIRMWARE_CONFIGS };
