idf_component_register(SRCS "can_demo_main.c" "fs.c" "obd.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi esp_netif esp_event fatfs http can)
