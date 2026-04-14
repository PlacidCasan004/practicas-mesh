#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

#define RX_SIZE              1500
#define TX_INTERVAL_MS       5000
#define MESH_MAX_LAYER       3

#define ROUTER_SSID          "WLAN_48"
#define ROUTER_PASS          "tequieromuchocartucho"
#define ROUTER_CHANNEL       2

#define MQTT_BROKER_URI      "mqtt://192.168.1.101"

/* Topic base */
#define MQTT_DATA_BASE_TOPIC "mesh/data"
#define MQTT_CMD_BASE_TOPIC  "mesh/cmd"

#define MQTT_QUEUE_LEN       10
#define MQTT_MSG_MAX_LEN     1500
#define MQTT_TOPIC_MAX_LEN   128

/* GPIO para simular actuador */
#define ACTUATOR_LED_GPIO    GPIO_NUM_2

typedef struct {
    char topic[MQTT_TOPIC_MAX_LEN];
    char payload[MQTT_MSG_MAX_LEN];
} mqtt_publish_item_t;

static QueueHandle_t mqtt_queue = NULL;

static const char *TAG = "MESH_TEST";
static const uint8_t MESH_ID[6] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};

static bool is_mesh_connected = false;
static bool mqtt_started = false;
static bool mqtt_connected = false;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static esp_netif_t *netif_sta = NULL;
static esp_netif_t *netif_ap = NULL;

/* Prototipos */
static void rx_task(void *arg);
static void tx_task(void *arg);
static void start_mesh_tasks_once(void);
static void mqtt_app_start(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void mqtt_task(void *arg);
static void mesh_init(void);

static void actuator_init(void);
static void process_mesh_command(const char *cmd);
static void format_mac_no_colons(const uint8_t mac[6], char *out, size_t out_size);
static bool parse_mac_from_suffix(const char *suffix, mesh_addr_t *addr);
static void forward_mqtt_command_to_mesh_topic(const char *topic, const char *payload);


/*
 * actuator_init:
 * Inicializa el GPIO del LED para simular un actuador.
 */
static void actuator_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ACTUATOR_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(ACTUATOR_LED_GPIO, 0);

    ESP_LOGI(TAG, "Actuador inicializado en GPIO %d", ACTUATOR_LED_GPIO);
}


/*
 * process_mesh_command:
 * Ejecuta una orden recibida por mesh en un nodo no-root.
 *
 * Comandos soportados:
 * - LED_ON
 * - LED_OFF
 */
static void process_mesh_command(const char *cmd)
{
    if (strcmp(cmd, "LED_ON") == 0) {
        gpio_set_level(ACTUATOR_LED_GPIO, 1);
        ESP_LOGI(TAG, "Actuador: LED encendido");
    } else if (strcmp(cmd, "LED_OFF") == 0) {
        gpio_set_level(ACTUATOR_LED_GPIO, 0);
        ESP_LOGI(TAG, "Actuador: LED apagado");
    } else {
        ESP_LOGW(TAG, "Comando mesh no reconocido: %s", cmd);
    }
}


/*
 * format_mac_no_colons:
 * Convierte una MAC a texto sin ':' para usarla en topics MQTT.
 *
 * Ejemplo:
 * 24:dc:c3:92:cc:94 -> 24dcc392cc94
 */
static void format_mac_no_colons(const uint8_t mac[6], char *out, size_t out_size)
{
    snprintf(out, out_size, "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


/*
 * parse_mac_from_suffix:
 * Convierte un sufijo tipo "24dcc392cc94" a mesh_addr_t.
 * Devuelve true si el formato es válido.
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
 * Recibe un topic tipo mesh/cmd/<mac> y un payload tipo LED_ON.
 * Extrae la MAC y reenvía la orden por mesh a ese nodo.
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
        ESP_LOGW(TAG, "Topic de comando no valido: %s", topic);
        return;
    }

    const char *mac_suffix = topic + prefix_len;
    mesh_addr_t dest;

    if (!parse_mac_from_suffix(mac_suffix, &dest)) {
        ESP_LOGW(TAG, "MAC invalida en topic: %s", topic);
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

    BaseType_t ok1 = xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, NULL);
    BaseType_t ok2 = xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);
    BaseType_t ok3 = xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);

    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS) {
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
 * rx_task:
 * Espera mensajes que llegan por la mesh.
 *
 * Si este nodo es root:
 * - publica el mensaje en mesh/data/<mac_origen>
 *
 * Si este nodo NO es root:
 * - interpreta comandos tipo LED_ON / LED_OFF
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
                if (mqtt_queue != NULL) {
                    mqtt_publish_item_t item;
                    char mac_str[32];

                    format_mac_no_colons(from.addr, mac_str, sizeof(mac_str));
                    snprintf(item.topic, sizeof(item.topic), "%s/%s", MQTT_DATA_BASE_TOPIC, mac_str);

                    size_t copy_len = data.size;
                    if (copy_len >= sizeof(item.payload)) {
                        copy_len = sizeof(item.payload) - 1;
                    }

                    memcpy(item.payload, data.data, copy_len);
                    item.payload[copy_len] = '\0';

                    if (xQueueSend(mqtt_queue, &item, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Cola MQTT llena, mensaje descartado");
                    } else {
                        ESP_LOGI(TAG, "Mensaje metido en cola MQTT para topic %s", item.topic);
                    }
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
 * - solo los nodos NO root envían
 * - envían hacia arriba con esp_mesh_send(NULL, ...)
 */
static void tx_task(void *arg)
{
    (void)arg;

    mesh_data_t data;
    char msg[100];
    uint8_t mac[6];

    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    data.data = (uint8_t *)msg;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    while (1) {
        if (is_mesh_connected && !esp_mesh_is_root()) {
            snprintf(msg, sizeof(msg), "Hola desde nodo " MACSTR " layer=%d",
                     MAC2STR(mac), esp_mesh_get_layer());

            data.size = strlen(msg) + 1;

            esp_err_t err = esp_mesh_send(NULL, &data, 0, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "TX hacia arriba: %s", msg);
            } else {
                ESP_LOGW(TAG, "Error TX: %s", esp_err_to_name(err));
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
                ESP_LOGW(TAG, "mqtt_task activa en un nodo no root, mensaje ignorado");
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

    actuator_init();
    mesh_init();
}
