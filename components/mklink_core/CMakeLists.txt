idf_component_register(
    SRCS
        "src/mklink_core.c"
        "src/mklink_wifi.c"
        "src/mklink_usb.c"  "src/mklink_discovery.c"  "src/mklink_probe.c"  "src/mklink_rtt.c"  "src/mklink_watch.c"  "src/mklink_stream_hub.c"
        "src/mklink_usb_raw.c"
        "src/mklink_cdc.c"
        "src/mklink_net.c"
        "src/mklink_usbip.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES usb
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_timer lwip json driver
)
