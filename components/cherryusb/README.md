# CherryUSB (trimmed host-only vendor drop)

Upstream: [cherry-embedded/CherryUSB](https://github.com/cherry-embedded/CherryUSB) v1.5.2
(taken from the ESP-IDF component registry package `cherry-embedded/cherryusb 1.5.2~2`).

Only what the MKLink USB-over-IP passthrough needs is vendored:

```
core/            usbh_core            host core (enumeration, hub port table, descriptors)
class/hub/       usbh_hub             HUB class driver (port reset, TT think, split scheduling)
common/          headers only
osal/idf/        usb_osal_idf.c, usb_config.h, linker.lf, usbh_class_info.ld
port/dwc2/       usb_hc_dwc2.c, usb_glue_esp.c, registers/params
```

Dropped from upstream: the whole device stack (`usbd_core.c`, `usbotg_core.c`,
`usb_dc_dwc2.c`, `common/usb_dc.h`, `common/usb_otg.h`), every class driver except
HUB, demos, docs, tests and the upstream Kconfig/CMake glue. The device-mode parts
of `port/dwc2/usb_glue_esp.c` were removed as well, so the component never touches
the USB device peripheral.

## Why vendor instead of depending on the registry component

The passthrough needs things the released component cannot provide:

* limits in `osal/idf/usb_config.h` raised for the 8-interface probe and for HUBs;
* device add/remove hooks (upstream has no public callback);
* a boot-time HUB port scan;
* a crash fix in the dwc2 channel interrupt handler.

## Patches against v1.5.2

| File | Change |
| --- | --- |
| `core/usbh_core.h/.c` | weak `usbh_device_connected_cb()` / `usbh_device_disconnected_cb()` hooks, called after enumeration and before a port is released |
| `class/hub/usbh_hub.c` | calls the connected hook; port reset+enumerate factored into `usbh_hub_port_connect()`; scans all ports once after HUB init (a HUB reports `C_PORT_CONNECTION` only once, so devices already attached at boot would otherwise be missed); clamps `bNbrPorts` to `CONFIG_USBHOST_MAX_EHPORTS` |
| `port/dwc2/usb_hc_dwc2.c` | `dwc2_inchan_irq_handler()` returns early when `chan->urb == NULL` (URB unlinked while a channel-halt interrupt was pending → `Load access fault`) |
| `osal/idf/usb_config.h` | `MAX_INTERFACES 8`, `MAX_ENDPOINTS 6`, `MAX_INTF_ALTSETTINGS 2`, `MAX_EHPORTS 4`, `MAX_EXTHUBS 1`, hub thread prio 5, control timeout 1 s, P4 `MAX_BUS 1` |
| `port/dwc2/usb_glue_esp.c` | device-mode glue removed (host only) |

Every port in `MAX_EHPORTS` costs ~1 KiB of `.bss` per HUB instance, and esp_hosted
allocates its WiFi TX buffers from the same internal RAM — see
`USBIP_PORTING.md` §"七、USB 主机栈迁移到 CherryUSB" before raising the limits.
