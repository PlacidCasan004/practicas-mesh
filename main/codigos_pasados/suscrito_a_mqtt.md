#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Cola para guardar los mensajes de la mesh y que luego la tarea de mqtt coja los mensajes de la cola para publicarlos al broker
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

#define RX_SIZE              1500
#define TX_INTERVAL_MS       5000
#define MESH_MAX_LAYER       3

#define ROUTER_SSID          "WLAN_48"
#define ROUTER_PASS          "tequieromuchocartucho"
#define ROUTER_CHANNEL       2

#define MQTT_BROKER_URI      "mqtt://192.168.1.101"
#define MQTT_TOPIC           "mesh/data"

#define MQTT_QUEUE_LEN       10
#define MQTT_MSG_MAX_LEN     1500

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


/*
 * start_mesh_tasks_once:
 * Arranca las tareas solo una vez.
 * - Crea la cola MQTT
 * - Crea rx_task
 * - Crea tx_task
 * - Crea mqtt_task
 */
static void start_mesh_tasks_once(void)
{
    static bool started = false;

    if (started) {
        return;
    }

    if (mqtt_queue == NULL) {
        mqtt_queue = xQueueCreate(MQTT_QUEUE_LEN, MQTT_MSG_MAX_LEN);
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
 * - Si conecta, pone mqtt_connected = true
 * - Si desconecta o falla, pone mqtt_connected = false
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT conectado");
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT desconectado");
            break;

        case MQTT_EVENT_ERROR:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT error");
            break;

        default:
            break;
    }
}


/*
 * mqtt_app_start:
 * Inicializa y arranca el cliente MQTT.
 * Solo debe llamarse cuando el root ya tiene IP.
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
 * Gestiona:
 * - eventos de la mesh
 * - eventos de IP
 *
 * Qué hace:
 * - Detecta cuando la mesh arranca
 * - Detecta cuando un nodo se conecta a su parent
 * - Si el nodo es root, arranca DHCP en la interfaz STA
 * - Arranca las tareas una sola vez
 * - Cuando el root consigue IP, arranca MQTT
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
 * Qué hace:
 * - Recibe mensajes con esp_mesh_recv()
 * - Imprime quién lo ha mandado
 * - Si este nodo es el root, copia el mensaje a la cola MQTT
 *
 * Importante:
 * - Aquí NO se publica directamente en MQTT
 * - Solo se mete en la cola
 */
static void rx_task(void *arg)
{
    (void)arg;

    // espera mensajes mesh, imprime quién lo manda y si el nodo es root, mete el contenido en la cola MQTT
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

            /* Solo el root pasa los mensajes a MQTT */
            if (esp_mesh_is_root() && mqtt_queue != NULL) {
                char mqtt_msg[MQTT_MSG_MAX_LEN];

                size_t copy_len = data.size;
                if (copy_len >= sizeof(mqtt_msg)) {
                    copy_len = sizeof(mqtt_msg) - 1;
                }

                memcpy(mqtt_msg, data.data, copy_len);
                mqtt_msg[copy_len] = '\0';

                if (xQueueSend(mqtt_queue, mqtt_msg, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Cola MQTT llena, mensaje descartado");
                } else {
                    ESP_LOGI(TAG, "Mensaje metido en cola MQTT");
                }
            }
        }
    }
}


/*
 * tx_task:
 * Envía mensajes por la mesh.
 *
 * En esta versión:
 * - solo los nodos NO root envían
 * - envían hacia arriba con esp_mesh_send(NULL, ...)
 * - eso hace que el mensaje suba hasta el root
 */
static void tx_task(void *arg)
{
    (void)arg;

    mesh_data_t data;
    char msg[100];
    uint8_t mac[6];

    /* Cambia esta MAC por la del nodo destino
    mesh_addr_t destino = {
        .addr = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}
    };
    */

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
 *
 * Qué hace:
 * - Espera a que rx_task meta mensajes en mqtt_queue
 * - Comprueba que este nodo es root
 * - Comprueba que el cliente MQTT existe
 * - Comprueba que MQTT está conectado
 * - Publica en el topic MQTT_TOPIC
 */
static void mqtt_task(void *arg)
{
    (void)arg;

    // espera mensajes en la cola y comprueba que este nodo sea root y MQTT esté conectado. Además, publica en el broker
    char mqtt_msg[MQTT_MSG_MAX_LEN];

    while (1) {
        if (mqtt_queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (xQueueReceive(mqtt_queue, mqtt_msg, portMAX_DELAY) == pdTRUE) {
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
                MQTT_TOPIC,
                mqtt_msg,
                0,
                1,
                0
            );

            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Publicado en MQTT topic %s, msg_id=%d, msg=%s",
                         MQTT_TOPIC, msg_id, mqtt_msg);
            } else {
                ESP_LOGW(TAG, "No se pudo publicar en MQTT");
            }
        }
    }
}


/*
 * mesh_init:
 * Función que monta toda la infraestructura.
 *
 * Qué hace:
 * - inicializa netif
 * - crea event loop
 * - crea interfaces mesh
 * - inicializa Wi-Fi
 * - inicializa mesh
 * - registra handlers
 * - configura mesh
 * - arranca mesh
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
 *
 * Qué hace:
 * - inicializa NVS
 * - imprime la MAC de esta placa
 * - arranca la mesh
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
