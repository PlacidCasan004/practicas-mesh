idf_component_register(SRCS "practicas.c"
                       INCLUDE_DIRS "."
                       REQUIRES bme280 mqtt esp_wifi nvs_flash)
