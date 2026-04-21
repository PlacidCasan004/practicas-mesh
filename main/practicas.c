#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mesh.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "node_labels.h"
#include "bme280.h"

#define RX_SIZE              512
#define TX_INTERVAL_MS       30000
#define SENSOR_INTERVAL_MS   2000
#define MESH_MAX_LAYER       3

#define ROUTER_SSID          "WLAN_48"
#define ROUTER_PASS          "tequieromuchocartucho"
#define ROUTER_CHANNEL       2

#define MQTT_BROKER_URI      "mqtt://192.168.1.101"

/* Topic base */
#define MQTT_DATA_BASE_TOPIC "mesh/data"
#define MQTT_CMD_BASE_TOPIC  "mesh/cmd"

#define MQTT_QUEUE_LEN       10
#define MQTT_MSG_MAX_LEN     512
#define MQTT_TOPIC_MAX_LEN   128

/* BME280 */
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define BME280_I2C_ADDRESS   0x76

/* Relé */
#define RELAY_PIN            GPIO_NUM_23
#define RELAY_ACTIVE_LOW     1

typedef struct {
    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_MSG_MAX_LEN];
} mqtt_publish_item_t;

typedef enum {
    RELAY_MODE_MANUAL_OFF = 0,
    RELAY_MODE_MANUAL_ON
} relay_mode_t;

typedef struct {
    float temperature;
    float pressure_hpa;
    float humidity;
    char payload[MQTT_MSG_MAX_LEN];
} sensor_snapshot_t;

static QueueHandle_t mqtt_queue = NULL;
static SemaphoreHandle_t sensor_mutex = NULL;
static SemaphoreHandle_t relay_mutex = NULL;

static const char *TAG = "MESH_TEST";
static const uint8_t MESH_ID[6] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

static bool is_mesh_connected = false;
static bool mqtt_started = false;
static bool mqtt_connected = false;
static bool sensor_ready = false;
static bool i2c_driver_installed = false;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static esp_netif_t *netif_sta = NULL;
static esp_netif_t *netif_ap = NULL;

static bme280_t bme280_dev;
static sensor_snapshot_t g_sensor_data = {0};
static relay_mode_t g_relay_mode = RELAY_MODE_MANUAL_OFF;

/* Prototipos */
static void rx_task(void *arg);
static void tx_task(void *arg);
static void mqtt_task(void *arg);
static void sensor_relay_task(void *arg);

static void start_mesh_tasks_once(void);
static void mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void mesh_init(void);

static void relay_init(void);
static void relay_set(bool on);
static void process_mesh_command(const char *cmd);

static void i2c_master_init_once(void);
static esp_err_t sensor_init_once(void);

static void format_mac_no_colons(const uint8_t mac[6], char *out, size_t out_size);
static bool parse_mac_from_suffix(const char *suffix, mesh_addr_t *addr);
static void forward_mqtt_command_to_mesh_topic(const char *topic, const char *payload);

static const char *get_node_id_from_mac_str(const char *mac_str);
static void enqueue_mqtt_message(const char *topic, const char *payload);
static bool split_bme280_payload_to_topics(const mesh_addr_t *from, const char *payload);

/*
 * relay_init:
 * Inicializa el GPIO del relé.
 */
static void relay_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Relé apagado al inicio */
    relay_set(false);

    ESP_LOGI(TAG, "Relé inicializado en GPIO %d", RELAY_PIN);
}

/*
 * relay_set:
 * Activa o desactiva el relé.
 */
static void relay_set(bool on)
{
    if (RELAY_ACTIVE_LOW) {
        gpio_set_level(RELAY_PIN, on ? 0 : 1);
    } else {
        gpio_set_level(RELAY_PIN, on ? 1 : 0);
    }
}

/*
 * i2c_master_init_once:
 * Inicializa I2C una sola vez.
 */
static void i2c_master_init_once(void)
{
    if (i2c_driver_installed) {
        return;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    i2c_driver_installed = true;
    ESP_LOGI(TAG, "I2C inicializado");
}

/*
 * sensor_init_once:
 * Inicializa el BME280 una sola vez.
 */
static esp_err_t sensor_init_once(void)
{
    if (sensor_ready) {
        return ESP_OK;
    }

    i2c_master_init_once();

    esp_err_t err;

    err = bme280_init_desc(&bme280_dev, BME280_I2C_ADDRESS, I2C_MASTER_NUM,
                           I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo inicializar descriptor BME280: %s", esp_err_to_name(err));
        return err;
    }

    err = bme280_init(&bme280_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME280 no detectado en este nodo: %s", esp_err_to_name(err));
        return err;
    }

    sensor_ready = true;
    ESP_LOGI(TAG, "BME280 inicializado correctamente");
    return ESP_OK;
}

/*
 * process_mesh_command:
 * Ejecuta una orden recibida por mesh.
 *
 * Comandos soportados:
 * - RELAY_ON
 * - RELAY_OFF
 */
static void process_mesh_command(const char *cmd)
{
    if (relay_mutex != NULL &&
        xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

        if (strcmp(cmd, "RELAY_ON") == 0) {
            g_relay_mode = RELAY_MODE_MANUAL_ON;
            relay_set(true);
            ESP_LOGI(TAG, "Comando recibido: RELAY_ON -> relé encendido");
        } else if (strcmp(cmd, "RELAY_OFF") == 0) {
            g_relay_mode = RELAY_MODE_MANUAL_OFF;
            relay_set(false);
            ESP_LOGI(TAG, "Comando recibido: RELAY_OFF -> relé apagado");
        } else {
            ESP_LOGW(TAG, "Comando mesh no reconocido: %s", cmd);
        }

        xSemaphoreGive(relay_mutex);
    } else {
        ESP_LOGW(TAG, "No se pudo tomar relay_mutex");
    }
}

/*
 * format_mac_no_colons:
 * Convierte una MAC a texto sin ':' para usarla en topics MQTT.
 */
static void format_mac_no_colons(const uint8_t mac[6], char *out, size_t out_size)
{
    snprintf(out, out_size, "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/*
 * get_node_id_from_mac_str:
 * Busca una MAC en node_labels.h.
 * Si existe, devuelve el identificador fácil.
 * Si no existe, devuelve la propia MAC.
 */
static const char *get_node_id_from_mac_str(const char *mac_str)
{
    for (size_t i = 0; i < NODE_LABELS_COUNT; i++) {
        if (strcmp(mac_str, NODE_LABELS[i].mac) == 0) {
            return NODE_LABELS[i].id;
        }
    }

    return mac_str;
}

/*
 * enqueue_mqtt_message:
 * Mete un mensaje en la cola MQTT del root.
 */
static void enqueue_mqtt_message(const char *topic, const char *payload)
{
    if (mqtt_queue == NULL) {
        ESP_LOGW(TAG, "Cola MQTT no inicializada");
        return;
    }

    mqtt_publish_item_t item;

    snprintf(item.topic, sizeof(item.topic), "%s", topic);
    snprintf(item.payload, sizeof(item.payload), "%s", payload);

    if (xQueueSend(mqtt_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Cola MQTT llena, mensaje descartado");
    } else {
        ESP_LOGI(TAG, "Mensaje metido en cola MQTT para topic %s", item.topic);
    }
}

/*
 * split_bme280_payload_to_topics:
 * Separa el JSON del BME280 en topics diferentes:
 *
 * mesh/data/<node_id>/temperature
 * mesh/data/<node_id>/pressure
 * mesh/data/<node_id>/humidity
 */
static bool split_bme280_payload_to_topics(const mesh_addr_t *from, const char *payload)
{
    char mac_str[32];
    const char *node_id;

    float temperature = 0.0f;
    float pressure_hpa = 0.0f;
    float humidity = 0.0f;
    int relay = 0;
    int layer = 0;
    char mode[32] = {0};

    format_mac_no_colons(from->addr, mac_str, sizeof(mac_str));
    node_id = get_node_id_from_mac_str(mac_str);

    int parsed = sscanf(payload,
                        "{\"temperature\":%f,\"pressure_hpa\":%f,\"humidity\":%f,\"relay\":%d,\"mode\":\"%31[^\"]\",\"layer\":%d}",
                        &temperature,
                        &pressure_hpa,
                        &humidity,
                        &relay,
                        mode,
                        &layer);

    if (parsed != 6) {
        ESP_LOGW(TAG, "No se pudo separar payload BME280: %s", payload);
        return false;
    }

    char topic[MQTT_TOPIC_MAX_LEN];
    char msg[MQTT_MSG_MAX_LEN];

    snprintf(topic, sizeof(topic), "%s/%s/temperature", MQTT_DATA_BASE_TOPIC, node_id);
    snprintf(msg, sizeof(msg),
             "{\"node\":\"%s\",\"sensor\":\"temperature\",\"value\":%.2f,\"unit\":\"C\",\"relay\":%d,\"mode\":\"%s\",\"layer\":%d}",
             node_id, temperature, relay, mode, layer);
    enqueue_mqtt_message(topic, msg);

    snprintf(topic, sizeof(topic), "%s/%s/pressure", MQTT_DATA_BASE_TOPIC, node_id);
    snprintf(msg, sizeof(msg),
             "{\"node\":\"%s\",\"sensor\":\"pressure\",\"value\":%.2f,\"unit\":\"hPa\",\"relay\":%d,\"mode\":\"%s\",\"layer\":%d}",
             node_id, pressure_hpa, relay, mode, layer);
    enqueue_mqtt_message(topic, msg);

    snprintf(topic, sizeof(topic), "%s/%s/humidity", MQTT_DATA_BASE_TOPIC, node_id);
    snprintf(msg, sizeof(msg),
             "{\"node\":\"%s\",\"sensor\":\"humidity\",\"value\":%.2f,\"unit\":\"%%\",\"relay\":%d,\"mode\":\"%s\",\"layer\":%d}",
             node_id, humidity, relay, mode, layer);
    enqueue_mqtt_message(topic, msg);

    return true;
}

/*
 * parse_mac_from_suffix:
 * Convierte un sufijo tipo "24dcc392cc94" a mesh_addr_t.
 */
static bool parse_mac_from_suffix(const char *suffix, mesh_addr_t *addr)
{
    if (suffix == NULL || strlen(suffix) != 12) {
        return false;
    }

    for (int i = 0; i < 12; i++) {
        if (!isxdigit((unsigned char)suffix[i])) {
            return false;
        }
    }

    unsigned int bytes[6];
    int n = sscanf(suffix, "%2x%2x%2x%2x%2x%2x",
                   &bytes[0], &bytes[1], &bytes[2],
                   &bytes[3], &bytes[4], &bytes[5]);
    if (n != 6) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        addr->addr[i] = (uint8_t)bytes[i];
    }

    return true;
}

/*
 * forward_mqtt_command_to_mesh_topic:
 * Recibe un topic tipo mesh/cmd/<mac> y un payload tipo RELAY_ON.
 * Extrae la MAC y reenvía la orden por mesh a ese nodo.
 *
 * Si el destino es el propio root, ejecuta la orden localmente.
 */
static void forward_mqtt_command_to_mesh_topic(const char *topic, const char *payload)
{
    if (!esp_mesh_is_root()) {
        ESP_LOGW(TAG, "forward_mqtt_command_to_mesh_topic llamado en nodo no-root");
        return;
    }

    const char *prefix = MQTT_CMD_BASE_TOPIC "/";
    size_t prefix_len = strlen(prefix);

    if (strncmp(topic, prefix, prefix_len) != 0) {
        ESP_LOGW(TAG, "Topic de comando no válido: %s", topic);
        return;
    }

    const char *mac_suffix = topic + prefix_len;
    mesh_addr_t dest;

    if (!parse_mac_from_suffix(mac_suffix, &dest)) {
        ESP_LOGW(TAG, "MAC inválida en topic: %s", topic);
        return;
    }

    uint8_t sta_mac[6];
    uint8_t ap_mac[6];

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, sta_mac));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, ap_mac));

    ESP_LOGI(TAG,
             "Destino comando=" MACSTR " | local STA=" MACSTR " | local AP=" MACSTR,
             MAC2STR(dest.addr),
             MAC2STR(sta_mac),
             MAC2STR(ap_mac));

    /*
     * Si el comando va dirigido al propio root, se ejecuta localmente.
     * Esto contempla tanto la MAC STA como la MAC AP.
     */
    if (memcmp(dest.addr, sta_mac, 6) == 0 ||
        memcmp(dest.addr, ap_mac, 6) == 0) {

        ESP_LOGI(TAG, "Comando MQTT dirigido al nodo ROOT local: %s", payload);
        process_mesh_command(payload);
        return;
    }

    mesh_data_t data;
    char msg[128];

    snprintf(msg, sizeof(msg), "%s", payload);

    data.data = (uint8_t *)msg;
    data.size = strlen(msg) + 1;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    esp_err_t err = esp_mesh_send(&dest, &data, MESH_DATA_FROMDS, NULL, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Comando MQTT reenviado por mesh a " MACSTR ": %s",
                 MAC2STR(dest.addr), msg);
    } else {
        ESP_LOGW(TAG, "Error reenviando comando por mesh: %s", esp_err_to_name(err));
    }
}

/*
 * start_mesh_tasks_once:
 * Arranca las tareas solo una vez.
 */
static void start_mesh_tasks_once(void)
{
    static bool started = false;

    if (started) {
        return;
    }

    if (mqtt_queue == NULL) {
        mqtt_queue = xQueueCreate(MQTT_QUEUE_LEN, sizeof(mqtt_publish_item_t));
        if (mqtt_queue == NULL) {
            ESP_LOGE(TAG, "No se pudo crear la cola MQTT");
            return;
        }
    }

    if (sensor_mutex == NULL) {
        sensor_mutex = xSemaphoreCreateMutex();
        if (sensor_mutex == NULL) {
            ESP_LOGE(TAG, "No se pudo crear el mutex del sensor");
            return;
        }
    }

    if (relay_mutex == NULL) {
        relay_mutex = xSemaphoreCreateMutex();
        if (relay_mutex == NULL) {
            ESP_LOGE(TAG, "No se pudo crear el mutex del rele");
            return;
        }
    }

    BaseType_t ok1 = xTaskCreate(rx_task, "rx_task", 12288, NULL, 5, NULL);
    BaseType_t ok2 = xTaskCreate(tx_task, "tx_task", 8192, NULL, 5, NULL);
    BaseType_t ok3 = xTaskCreate(mqtt_task, "mqtt_task", 8192, NULL, 5, NULL);
    BaseType_t ok4 = xTaskCreate(sensor_relay_task, "sensor_relay_task", 8192, NULL, 5, NULL);
    
    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS) {
        ESP_LOGE(TAG, "No se pudieron crear todas las tareas");
        return;
    }

    started = true;
    ESP_LOGI(TAG, "Tareas creadas correctamente");
}

/*
 * mqtt_event_handler:
 * Gestiona los eventos del cliente MQTT.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT conectado");

            if (esp_mesh_is_root()) {
                int msg_id = esp_mqtt_client_subscribe(mqtt_client, MQTT_CMD_BASE_TOPIC "/#", 1);
                ESP_LOGI(TAG, "Suscrito a topic %s/#, msg_id=%d", MQTT_CMD_BASE_TOPIC, msg_id);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT desconectado");
            break;

        case MQTT_EVENT_ERROR:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT error");
            break;

        case MQTT_EVENT_DATA: {
            char topic[MQTT_TOPIC_MAX_LEN];
            char payload[256];

            int topic_len = event->topic_len;
            int data_len = event->data_len;

            if (topic_len >= (int)sizeof(topic)) {
                topic_len = sizeof(topic) - 1;
            }
            if (data_len >= (int)sizeof(payload)) {
                data_len = sizeof(payload) - 1;
            }

            memcpy(topic, event->topic, topic_len);
            topic[topic_len] = '\0';

            memcpy(payload, event->data, data_len);
            payload[data_len] = '\0';

            ESP_LOGI(TAG, "MQTT DATA topic=%s payload=%s", topic, payload);

            if (esp_mesh_is_root()) {
                forward_mqtt_command_to_mesh_topic(topic, payload);
            }
            break;
        }

        default:
            break;
    }
}

/*
 * mqtt_app_start:
 * Inicializa y arranca el cliente MQTT.
 */
static void mqtt_app_start(void)
{
    if (mqtt_started) {
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "No se pudo crear el cliente MQTT");
        return;
    }

    ESP_ERROR_CHECK(
        esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL)
    );

    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    mqtt_started = true;
    ESP_LOGI(TAG, "MQTT iniciado");
}

/*
 * mesh_event_handler:
 * Gestiona eventos de mesh e IP.
 */
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == MESH_EVENT) {
        switch (event_id) {
            case MESH_EVENT_STARTED:
                ESP_LOGI(TAG, "Mesh iniciada");
                break;

            case MESH_EVENT_PARENT_CONNECTED:
                is_mesh_connected = true;

                ESP_LOGI(TAG, "Conectado a mesh (%s), layer=%d",
                         esp_mesh_is_root() ? "ROOT" : "NODE",
                         esp_mesh_get_layer());
		mesh_addr_t parent_addr;
		esp_mesh_get_parent_bssid(&parent_addr);

		ESP_LOGI(TAG, "Parent BSSID: " MACSTR, MAC2STR(parent_addr.addr));
		
                if (esp_mesh_is_root() && netif_sta != NULL) {
                    esp_err_t err;

                    err = esp_netif_dhcpc_stop(netif_sta);
                    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
                        ESP_LOGW(TAG, "No se pudo parar DHCP cliente: %s", esp_err_to_name(err));
                    }

                    err = esp_netif_dhcpc_start(netif_sta);
                    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
                        ESP_LOGW(TAG, "No se pudo arrancar DHCP cliente: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "DHCP cliente arrancado en root");
                    }
                }

                start_mesh_tasks_once();
                break;

            case MESH_EVENT_PARENT_DISCONNECTED:
                is_mesh_connected = false;
                ESP_LOGW(TAG, "Desconectado de mesh");
                break;

            case MESH_EVENT_LAYER_CHANGE:
                ESP_LOGI(TAG, "Cambio de capa: %d", esp_mesh_get_layer());
                break;

            case MESH_EVENT_ROOT_SWITCH_REQ:
                ESP_LOGW(TAG, "Cambio de ROOT solicitado");
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

                ESP_LOGI(TAG, "ROOT tiene IP: " IPSTR, IP2STR(&event->ip_info.ip));

                if (esp_mesh_is_root()) {
                    ESP_LOGI(TAG, "Arrancando MQTT (ya con IP)");
                    mqtt_app_start();
                }
                break;
            }

            default:
                break;
        }
    }
}

/*
 * sensor_relay_task:
 * Lee el BME280 y actualiza el estado global.
 * El relé NO depende de la temperatura.
 * El relé solo cambia cuando llega RELAY_ON o RELAY_OFF.
 */
static void sensor_relay_task(void *arg)
{
    (void)arg;

    esp_err_t err = sensor_init_once();
    relay_init();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Este nodo no tiene BME280. Seguira en mesh sin sensor.");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        float temperature = 0.0f;
        float pressure = 0.0f;
        float humidity = 0.0f;

        esp_err_t res = bme280_read_float_data(&bme280_dev, &temperature, &pressure, &humidity);

        if (res == ESP_OK) {
            float pressure_hpa = pressure / 100.0f;
            relay_mode_t current_mode = RELAY_MODE_MANUAL_OFF;
            const char *mode_str = "MANUAL_OFF";
            int relay_state = 0;

            if (relay_mutex != NULL &&
                xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_mode = g_relay_mode;
                xSemaphoreGive(relay_mutex);
            }

            if (current_mode == RELAY_MODE_MANUAL_ON) {
                relay_set(true);
                mode_str = "MANUAL_ON";
                relay_state = 1;
            } else {
                relay_set(false);
                mode_str = "MANUAL_OFF";
                relay_state = 0;
            }

            if (sensor_mutex != NULL &&
                xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

                g_sensor_data.temperature = temperature;
                g_sensor_data.pressure_hpa = pressure_hpa;
                g_sensor_data.humidity = humidity;

                snprintf(g_sensor_data.payload, sizeof(g_sensor_data.payload),
                         "{\"temperature\":%.2f,\"pressure_hpa\":%.2f,\"humidity\":%.2f,\"relay\":%d,\"mode\":\"%s\",\"layer\":%d}",
                         temperature, pressure_hpa, humidity, relay_state, mode_str, esp_mesh_get_layer());

                xSemaphoreGive(sensor_mutex);
            }

            ESP_LOGI(TAG,
                     "BME280 -> T=%.2f C | P=%.2f hPa | H=%.2f %% | Relay=%d | Mode=%s",
                     temperature,
                     pressure_hpa,
                     humidity,
                     relay_state,
                     mode_str);
        } else {
            ESP_LOGE(TAG, "Error leyendo BME280");
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
    }
}

/*
 * rx_task:
 * Espera mensajes que llegan por la mesh.
 *
 * Si este nodo es root:
 * - separa mensajes BME280 en topics diferentes.
 *
 * Si este nodo NO es root:
 * - interpreta comandos tipo RELAY_ON / RELAY_OFF.
 */
static void rx_task(void *arg)
{
    (void)arg;

    mesh_addr_t from;
    mesh_data_t data;
    uint8_t rx_buf[RX_SIZE];
    int flag = 0;
    esp_err_t err;

    data.data = rx_buf;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    while (1) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_mesh_recv error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (data.size > 0) {
            size_t copy_len_rx = data.size;
            if (copy_len_rx >= RX_SIZE) {
                copy_len_rx = RX_SIZE - 1;
            }
            rx_buf[copy_len_rx] = '\0';

            ESP_LOGI(TAG, "RX desde " MACSTR ": %s",
                     MAC2STR(from.addr), (char *)data.data);

            if (esp_mesh_is_root()) {
                /*
                 * Si el root recibe por mesh un comando de control,
                 * no lo publica como dato.
                 */
                if (strcmp((char *)data.data, "RELAY_ON") == 0 ||
                    strcmp((char *)data.data, "RELAY_OFF") == 0) {
                    ESP_LOGI(TAG, "Root recibió comando de control desde mesh, no se publica como dato: %s",
                             (char *)data.data);
                    continue;
                }

                /*
                 * Si el mensaje es un JSON del BME280, se separa por medición.
                 */
                bool separated = split_bme280_payload_to_topics(&from, (char *)data.data);

                if (!separated) {
                    /*
                     * Fallback: si no se puede separar, se publica en raw usando etiqueta.
                     */
                    char mac_str[32];
                    const char *node_id;
                    char topic[MQTT_TOPIC_MAX_LEN];

                    format_mac_no_colons(from.addr, mac_str, sizeof(mac_str));
                    node_id = get_node_id_from_mac_str(mac_str);

                    snprintf(topic, sizeof(topic), "%s/%s/raw", MQTT_DATA_BASE_TOPIC, node_id);

                    enqueue_mqtt_message(topic, (char *)data.data);
                }
            } else {
                process_mesh_command((char *)data.data);
            }
        }
    }
}

/*
 * tx_task:
 * Envía mensajes por la mesh.
 *
 * - solo los nodos NO root envían.
 * - envían hacia arriba con esp_mesh_send(NULL, ...).
 * - se envía el último snapshot del BME280.
 */
static void tx_task(void *arg)
{
    (void)arg;

    mesh_data_t data;
    char msg[MQTT_MSG_MAX_LEN];

    data.data = (uint8_t *)msg;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    while (1) {
        if (is_mesh_connected && sensor_ready) {
            bool has_data = false;

            if (sensor_mutex != NULL &&
                xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                snprintf(msg, sizeof(msg), "%s", g_sensor_data.payload);
                xSemaphoreGive(sensor_mutex);
                has_data = (strlen(msg) > 0);
            }

            if (has_data) {
                if (!esp_mesh_is_root()) {
                    data.size = strlen(msg) + 1;

                    esp_err_t err = esp_mesh_send(NULL, &data, 0, NULL, 0);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "TX sensor hacia arriba: %s", msg);
                    } else {
                        ESP_LOGW(TAG, "Error TX: %s", esp_err_to_name(err));
                    }
                } else {
                    mesh_addr_t self_addr;
                    uint8_t mac[6];

                    esp_read_mac(mac, ESP_MAC_WIFI_STA);
                    memcpy(self_addr.addr, mac, 6);

                    ESP_LOGI(TAG, "ROOT publica su propio sensor: %s", msg);

                    bool separated = split_bme280_payload_to_topics(&self_addr, msg);

                    if (!separated) {
                        char mac_str[32];
                        const char *node_id;
                        char topic[MQTT_TOPIC_MAX_LEN];

                        format_mac_no_colons(self_addr.addr, mac_str, sizeof(mac_str));
                        node_id = get_node_id_from_mac_str(mac_str);

                        snprintf(topic, sizeof(topic), "%s/%s/raw", MQTT_DATA_BASE_TOPIC, node_id);
                        enqueue_mqtt_message(topic, msg);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}
/*
 * mqtt_task:
 * Espera mensajes en la cola y los publica en el broker MQTT.
 */
static void mqtt_task(void *arg)
{
    (void)arg;

    mqtt_publish_item_t item;

    while (1) {
        if (mqtt_queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (xQueueReceive(mqtt_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (!esp_mesh_is_root()) {
                ESP_LOGW(TAG, "mqtt_task activa en un nodo no-root, mensaje ignorado");
                continue;
            }

            if (mqtt_client == NULL) {
                ESP_LOGW(TAG, "Cliente MQTT no inicializado");
                continue;
            }

            if (!mqtt_connected) {
                ESP_LOGW(TAG, "MQTT no conectado, no se publica");
                continue;
            }

            int msg_id = esp_mqtt_client_publish(
                mqtt_client,
                item.topic,
                item.payload,
                0,
                1,
                0
            );

            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Publicado en MQTT topic %s, msg_id=%d, msg=%s",
                         item.topic, msg_id, item.payload);
            } else {
                ESP_LOGW(TAG, "No se pudo publicar en MQTT");
            }
        }
    }
}

/*
 * mesh_init:
 * Función que monta toda la infraestructura.
 */
static void mesh_init(void)
{
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_netif_create_default_wifi_mesh_netifs(&netif_sta, &netif_ap);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_wifi_init(&wifi_init_config);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t sta_mac[6];
    uint8_t ap_mac[6];

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, sta_mac));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, ap_mac));

    ESP_LOGI(TAG, "MAC STA: " MACSTR, MAC2STR(sta_mac));
    ESP_LOGI(TAG, "MAC AP : " MACSTR, MAC2STR(ap_mac));

    ESP_ERROR_CHECK(esp_mesh_init());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &mesh_event_handler,
                                               NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &mesh_event_handler,
                                               NULL));

    memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);

    cfg.channel = ROUTER_CHANNEL;

    cfg.router.ssid_len = strlen(ROUTER_SSID);
    memcpy((uint8_t *)&cfg.router.ssid, ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *)&cfg.router.password, ROUTER_PASS, strlen(ROUTER_PASS));

    cfg.mesh_ap.max_connection = 6;
    strcpy((char *)cfg.mesh_ap.password, "12345678");

    ESP_ERROR_CHECK(esp_mesh_set_max_layer(MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA2_PSK));

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, true));
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "Mesh arrancada");
}

/*
 * app_main:
 * Punto de entrada del programa.
 */
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Mi MAC es: " MACSTR, MAC2STR(mac));

    mesh_init();
}
