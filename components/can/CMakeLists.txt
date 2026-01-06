idf_component_register(SRCS "CAN.c"
                    INCLUDE_DIRS "include" "."
                    REQUIRES driver esp_driver_twai freertos)
