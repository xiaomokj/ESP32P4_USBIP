# Copilot Instructions

## Architecture

- This is ESP-IDF 5.5.5 firmware for the Waveshare ESP32-P4-WIFI6-Touch-LCD-4B. The P4 runs the USB host, service logic, and LVGL/Brookesia UI; its ESP32-C6 co-processor provides Wi-Fi over ESP-Hosted SDIO.
- `main/main.cpp` initializes NVS and `mklink_core`, brings up the display and phone shell, installs the apps, then starts core services. Keep service startup after UI setup unless intentionally changing boot behavior.
- `components/mklink_core` owns shared state and services: Wi-Fi, USB enumeration/raw transfers, USB/IP, discovery, networking, and logging. Its public C APIs are in `include/`; implementations and private headers are in `src/`.
- `components/apps` contains the Brookesia/LVGL applications. `UsbipApp` and `MkLinkUsbipApp` are separate UIs for the same core USB/IP service. To add an app, expose its header through `components/apps/apps.h` and install it in `main/main.cpp`.
- The ESP-IDF component manifest and lockfile are `main/idf_component.yml` and `dependencies.lock`. The P4's `esp_hosted` dependency is pinned to 2.12.13; the C6 slave firmware must use the matching version.

See [README.md](../README.md) for hardware setup and operation, and [USBIP_PORTING.md](../USBIP_PORTING.md) for USB/IP protocol and host-controller constraints.

## Build and Validation

Use ESP-IDF 5.5.5:

```bash
tools/build.sh                    # build P4 firmware
tools/build.sh flash [PORT]       # build and flash; default port /dev/ttyACM0
tools/build.sh monitor [PORT]     # serial monitor at 921600 baud
tools/build.sh fullflash [PORT]   # erase flash, then build and flash
tools/c6_build.sh                 # build the ESP32-C6 hosted slave firmware
tools/c6_flash.sh [PORT]          # flash the C6 after putting it in download mode
```

`tools/build.sh` uses `IDF_ACTIVATE` (default: `/root/.espressif/tools/activate_idf_v5.5.5.sh`) and unsets `IDF_COMPONENT_LOCAL_STORAGE_URL` for component resolution. Set `IDF_ACTIVATE` if the local IDF activation script is elsewhere. The project scripts and README target IDF 5.5.5, while `.vscode/settings.json` currently selects IDF 6.1; use the wrapper for a consistent project build unless intentionally migrating the IDF version. There is no repository-level automated test or lint target; hardware checks are documented in the README.

## Repository Conventions

- `components/apps/CMakeLists.txt` recursively collects app sources; `components/mklink_core/CMakeLists.txt` lists core sources explicitly, so add new core source files there. Keep public core APIs in `include/` and private implementation details in `src/`.
- `mklink_wifi_*` is the sole owner of `esp_wifi`. UI and settings code must use that API rather than managing Wi-Fi directly.
- USB discovery caches descriptors without claiming interfaces. USB/IP takes exclusive ownership during import; keep `CONFIG_MKLINK_PROBE_ENABLE` and `CONFIG_MKLINK_CDC_TEST_ENABLE` disabled for USB/IP operation. A pending bulk-IN transfer may not time out on this host controller, so preserve the existing transfer abort and interface teardown behavior.
- Core state is exposed through copy-out getters and events. Keep `lv_*` calls on the LVGL task; the dashboards refresh from timers and should use the `*_if_changed` helpers in `MkLinkUi.hpp`. Use `mklink_log()` for messages that should appear in both the console and the dashboard log.
- `mklink_usbip.c` implements the Linux USB/IP wire format. Preserve its packed 48-byte header, 312-byte device record, and network byte order. The successful import reply is exactly 320 bytes (`op_common` plus the device record); descriptors are fetched later via USB requests. Keep `TCP_NODELAY` and the short data-plane poll. Consult `USBIP_PORTING.md` before changing protocol or transfer handling.
- `sdkconfig.defaults` contains board-critical settings, including ESP32-P4 v1 silicon support, a 1024-byte USB control-transfer limit, 4-bit/40 MHz hosted SDIO, and disabled hosted SDIO mempool. Preserve these, along with the probe/CDC USB ownership settings, unless the hardware or operating mode is changing.
- The MKLink dashboards use the checked-in LVGL CJK font `components/apps/mklink_usbip/assets/mklink_font_cjk_16.c`. Maintain glyph coverage when changing Chinese UI text. Verify `tools/gen_font.py` paths before using it; its source globs and output path still refer to the older `components/apps/mklink/` layout.
