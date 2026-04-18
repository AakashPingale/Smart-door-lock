idf_component_register(SRCS "home_automation.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos nvs_flash esp_timer)