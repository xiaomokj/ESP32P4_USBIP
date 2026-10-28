# CherryUSB (trimmed host-only vendor drop) for MKLink Remote
#
# Only the host stack, the hub class and the ESP32-P4 dwc2 port are vendored:
# the project uses CherryUSB as a raw "enumerate + submit URB" engine for the
# usbip server, so no other class driver is compiled (and none can claim a
# device that has to stay available to the passthrough).
#
# Differences against the upstream 1.5.2 component (see cherryusb/README.md):
#   - device-mode sources and the USB device glue are removed;
#   - osal/idf/usb_config.h raises the interface/endpoint/port limits and
#     lowers the hub thread priority;
#   - core/usbh_core.c + class/hub/usbh_hub.c call the weak
#     usbh_device_{connected,disconnected}_cb() hooks.

idf_component_register(
    SRCS
        "core/usbh_core.c"
        "class/hub/usbh_hub.c"
        "port/dwc2/usb_hc_dwc2.c"
        "port/dwc2/usb_glue_esp.c"
        "osal/idf/usb_osal_idf.c"
    INCLUDE_DIRS
        "."
        "common"
        "core"
        "class/hub"
        "osal/idf"
        "port/dwc2"
    PRIV_REQUIRES
        usb
        esp_timer
        esp_mm
    LDFRAGMENTS
        "osal/idf/linker.lf"
)

# The class-info table lives in its own linker section; keep it referenced.
target_linker_script(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_LIST_DIR}/osal/idf/usbh_class_info.ld")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u hub_class_info")
