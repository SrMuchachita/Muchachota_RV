idf_component_register(SRCS "main2.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        nvs_flash
                        bt
                        esp_event
                        app_update
                        esp_app_format
                        esp_timer
                        esp_driver_gptimer
                        esp_driver_i2c
                        esp_driver_mcpwm
                        esp_driver_uart
                        esp_driver_gpio
                        modbus_params
                        servo_motor)
