idf_component_register(
    SRCS "practicas.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_event esp_netif esp_wifi mqtt driver json bme280
)
