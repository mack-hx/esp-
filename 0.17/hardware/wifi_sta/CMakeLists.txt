idf_component_register(SRCS "wifi_sta.c" "wifi_provision_page.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_netif esp_event lwip
                             nvs_flash esp_http_server esp_timer)
