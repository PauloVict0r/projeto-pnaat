idf_component_register(SRCS "hello_world_main.c" "imu_config.c"
                       PRIV_REQUIRES spi_flash bno085 i2c_config oled_setup oled_printf
                       INCLUDE_DIRS "")
