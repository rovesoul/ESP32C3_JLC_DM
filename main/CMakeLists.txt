idf_component_register(SRCS "NtcAdcScreenDHT22.c" "ntc.c" "dht22.c" "OLED.c" "OLED_Data.c" "simple_wifi_sta.c" "wifi_provisioning.c"
                    REQUIRES
                     driver
                     esp_driver_ledc
                     esp_driver_gpio
                     esp_adc
                    PRIV_REQUIRES
                     freertos
                     esp_http_server
                     nvs_flash
                     esp_netif
                     esp_wifi
                     esp_event
                     esp_timer

                     INCLUDE_DIRS "."
                     EMBED_TXTFILES "index.html"
                     )