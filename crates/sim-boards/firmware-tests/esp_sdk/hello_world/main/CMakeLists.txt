# ESP-IDF v6.1.1, examples/get-started/hello_world/main/CMakeLists.txt.
idf_component_register(SRCS "hello_world_main.c" PRIV_REQUIRES spi_flash INCLUDE_DIRS "")
