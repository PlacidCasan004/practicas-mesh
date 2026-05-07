#include "esp_log.h"

#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "model/barcos_model.h"

static const char *TAG = "BARCOS_MODEL";

extern "C" void test_barcos_model_load(void)
{
    const tflite::Model *model = tflite::GetModel(barcos_model_tflite);

    ESP_LOGI(TAG, "Modelo barcos cargado");
    ESP_LOGI(TAG, "Tamaño modelo: %u bytes", barcos_model_tflite_len);
    ESP_LOGI(TAG, "Schema version modelo: %ld", model->version());
    ESP_LOGI(TAG, "Schema version esperada: %d", TFLITE_SCHEMA_VERSION);

    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Version de schema incompatible");
        return;
    }

    ESP_LOGI(TAG, "Modelo TFLite reconocido correctamente");
}
