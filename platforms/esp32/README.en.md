> **Language:** English · [中文](README.md)

> English edition of the Chinese document. The Chinese file is authoritative if the two differ.

# Waveshare ESP32-S3-Touch-AMOLED-2.16 firmware

For a first flash see the [complete Waveshare edition DIY guide](../../docs/WAVESHARE_DIY_GUIDE.en.md),
and for operation once installed see the [user manual](../../docs/USER_MANUAL.en.md).

Targets the **ESP32-S3-Touch-AMOLED-2.16**: CO5300 QSPI, CST9220, 480×480, 16 MB Flash,
8 MB PSRAM. It uses ESP-IDF **5.5.5**, the Waveshare 2.16 BSP and the pinned LVGL submodule.

## Build without flashing the board

First prepare and activate an ESP-IDF 5.5.5 environment following Espressif's installation
instructions, then run this in the repository root:

```sh
git submodule update --init --recursive
idf.py -C platforms/esp32 set-target esp32s3
idf.py -C platforms/esp32 build
```

The first build downloads the Component Manager dependencies; the output is in `platforms/esp32/build/`.
In `dependencies.lock` LVGL is a relative path under the project root; if a local tool rewrites it to
an absolute path, do not commit your personal paths back to the repository. Do not modify
`managed_components` by hand to keep the build going.

## Backup and flashing

Flashing replaces the factory application. First confirm the exact board model, USB serial port, Flash
capacity and encryption/secure boot state, and save a complete factory backup yourself; a backup may
contain device credentials, so **do not upload it to GitHub**.
The PORT below is a placeholder, replace it with the actual USB serial port; the commands need the
Python environment of an activated IDF.

```sh
python -m esptool --chip esp32s3 --port PORT flash_id
python -m esptool --chip esp32s3 --port PORT get_security_info
mkdir -p backups
# Only for boards confirmed to be 16 MB with Flash encryption/secure boot not enabled:
python -m esptool --chip esp32s3 --port PORT read_flash 0 0x1000000 backups/waveshare-original.bin
```

The syntax above corresponds to esptool 4.x in an ESP-IDF 5.5.5 environment. If you use esptool 5.x
yourself, the subcommands become `flash-id` / `get-security-info` / `read-flash`; rely on that version's
`--help`.
Stop if the security state, capacity or model does not match; the backup file should be 16,777,216
bytes, and save the check value separately.
Only after the backup succeeds and you accept overwriting the factory firmware, run:

```sh
idf.py -C platforms/esp32 -p PORT flash monitor
```

`erase-flash` is not needed; leave the serial log with Ctrl+]. Use only the flash parameters generated
by this local build, and do not apply another board's image offsets. On first use complete the system
Bluetooth pairing in the iPhone.

## Behaviour and limitations

- Advertised name `MOTO GPS`; the iPhone provides positioning, routes, scene and music state over BLE.
- Page order: navigation, speedometer, trip, heading, clock, music. Trip figures are accumulated and
  sent by the phone, and a long press clears them.
- The clock page prefers the time the phone pushes. Once the phone is gone the board's own
  PCF85063ATL (address 0x51 on the shared I2C bus) keeps running, so the page is restored from the
  chip at boot instead of falling back to `--:--`. A phone time more than 90 seconds away is written
  back to correct the chip. The PCF85063 stores a two-digit year and the protocol carries no year, so
  while the chip runs alone the century comes from the firmware's build year; the first phone push
  restores the real year.
- After the engine stops and the chassis is quiet for 45 seconds (180 seconds while navigating) the
  panel blanks and rendering pauses; chassis vibration, phone speed, touch or the PWR key wake it.
- Phone notifications (ANCS) show calls, SMS and app alerts full screen. The board stays a BLE
  peripheral: ANCS rides the connection the phone opened, and the board is a GATT client on that same
  link, so the navigation protocol is unaffected. iOS only publishes ANCS to an authorized (bonded)
  link, so delivery depends on iPhone pairing and notification permissions; when it is not published
  the log records `ANCS is not published on this link` and navigation is unaffected.
- Notification titles and bodies use fonts shipped in the firmware:
  `shared/nav_ui/assets/moto_font_notify_16.c` and `moto_font_notify_20.c` cover the GB2312 level-1
  Hanzi set (about 99.7% of everyday Chinese) and are produced by `scripts/generate_fonts.sh`; rare
  characters fall back to placeholder boxes.
- After the black-and-white boot animation it enters the connection page; once connected it waits for
  the phone to select a route and does not draw an empty route as `0 m`.
- Swipe left and right to change pages; the page dots hide after five seconds. Whether the music page
  is available depends on the capabilities the phone declares.
- Holding the active-low GPIO18 user key for about three seconds requests shutdown from the AXP2101; on USB power there is a
  deep-sleep fallback. Different power supply combinations still need to be measured.
- The QMI8658 relative angular rate assists turning; the heading while moving is anchored by phone
  positioning, and no absolute north reference at rest is provided.
- PSRAM double buffering is enabled. The 2.16 board does not route CO5300 TE to the ESP32, so
  the 1.75C GPIO13 TE synchroniser is disabled.
- `sdkconfig.defaults` enables `CONFIG_BT_NIMBLE_ROLE_CENTRAL` and `CONFIG_BT_NIMBLE_GATT_CLIENT`:
  the board does not use the central role (it never scans or initiates a connection); the option is
  only on because ESP-IDF's GATT client support depends on it and ANCS needs the GATT client.
  `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` stays at 1.

Key pins: QSPI D0–D3 are GPIO4–7; SCLK38, CS12, RESET39; I2C SDA15/SCL14;
touch INT11/RESET40; user key GPIO18. Do not treat these pins as free GPIOs you can connect to anything.

Upstream: [Waveshare 2.16 examples](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.16),
[Waveshare BSP component](https://components.espressif.com/components/waveshare/esp32_s3_touch_amoled_2_16).
For the known background disconnection and route issues see `docs/KNOWN_ISSUES.md` in the repository
root. A successful build does not mean riding acceptance has passed.
