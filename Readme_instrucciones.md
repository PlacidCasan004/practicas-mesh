🚀 PASO 1 — Ir al proyecto
cd ~/esp-idf/practicas


⚙️ PASO 2 — Cargar entorno (MUY IMPORTANTE)
👉 Esto hay que hacerlo en cada terminal
. ~/esp-idf/export.sh


🎯 PASO 3 — Seleccionar chip (solo la primera vez)
(Solo si no lo hiciste antes)
idf.py set-target esp32


🧹 PASO 4 — Limpiar (recomendado tras cambios)
idf.py fullclean


🔨 PASO 5 — Compilar
idf.py build


🔌 PASO 6 — Flashear las placas

idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB1 flash
idf.py -p /dev/ttyUSB2 flash
idf.py -p /dev/ttyUSB3 flash


👀 PASO 7 — Abrir monitores (MUY IMPORTANTE)
🖥️ Terminal 1
. ~/esp-idf/export.sh
cd ~/esp-idf/practicas
idf.py -p /dev/ttyUSB0 monitor

🖥️ Terminal 2
. ~/esp-idf/export.sh
cd ~/esp-idf/practicas
idf.py -p /dev/ttyUSB1 monitor


🖥️ Terminal 3
. ~/esp-idf/export.sh
cd ~/esp-idf/practicas
idf.py -p /dev/ttyUSB2 monitor



mosquitto_sub -h 192.168.1.102 -t "#"



Actualmente, se elige el nodo destino de la orden del "actuador" según la mac y según se publique en el topic de la mac adecuada



Fase 1. Entender la red mesh. Ver si se pueden conectar entre sí los nodos


Fase 2. Comunicación con datos reales
- Conectar el ESP32 con el sensor y enviar información a MQTT
- Suscribirme desde los ESP32 a un topic y publicar desde fuera


Fase 3. Visualización y almacenamiento
- Node-RED:
	suscribirte al broker MQTT
	ver los datos de forma más visual
	filtrarlos
	mandarlos a una base de datos
	visualizar en dashboard
- Base de datos y procesamiento

Fase 4. Organización del firmware
- Entender bien para qué sirven las tareas

Fase 5. Entorno completo
- Docker con base de datos para procesar cadenas y datos



QUE NODO SE ELIJE COMO DESTINO DE LA ORDEN DEL ACTUADOR 

Ahora mismo, con el código que tienes, la orden se envía al último nodo que haya mandado algo al root.

Por qué

Porque en rx_task() del root haces esto conceptualmente:

llega un mensaje desde un nodo
guardas su MAC en last_node_addr
marcas last_node_known = true

O sea, cada vez que el root recibe un heartbeat tipo:

Hola desde nodo 24:dc:c3:8e:2f:48 layer=2

actualiza el destino.

