idf_component_register(
    SRCS "esp_peripherals.c"
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_uart esp_hw_support
)
