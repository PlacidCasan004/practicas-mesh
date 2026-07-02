# Variantes de firmware utilizadas

Durante el proyecto se han utilizado dos variantes de firmware según la capacidad de memoria de cada placa.

## ESP32 4 MB - Sensor-actuador

Para las placas ESP32 con 4 MB de memoria flash se utiliza una versión ligera del firmware, sin cámara ni modelo de inteligencia artificial.

Esta versión incluye:

- Comunicación ESP-MESH.
- Configuración dinámica por MAC.
- Comunicación MQTT.
- Lectura del BME280 por I2C.
- Control del relé en GPIO23.

Configuración hardware:

- SDA: GPIO21
- SCL: GPIO22
- Relé: GPIO23
- Flash: 4 MB
- Tabla de particiones: partitions_4mb.csv

## ESP32-S3 - Cámara e IA

Para el nodo cámara se utiliza la versión completa del firmware, que incluye:

- Captura de imagen.
- Integración del modelo TensorFlow Lite.
- Inferencia de barcos.
- Publicación de resultados mediante MQTT.

Esta versión requiere una placa ESP32-S3 con mayor capacidad de memoria.

