idf_component_register(
    SRCS         "wifi_mgr.c" "mqtt_cli.c" "net_app.c"
    INCLUDE_DIRS "include" "."
    REQUIRES     esp_wifi esp_event esp_netif mqtt nvs_flash lwip
    PRIV_REQUIRES app_data json ota_app esp_app_format
)