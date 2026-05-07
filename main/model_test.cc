#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "img_converters.h"
#include "mbedtls/base64.h"

#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "model/barcos_model.h"

static const char *TAG_MODEL = "BARCOS_MODEL";

#define TENSOR_ARENA_SIZE (4 * 1024 * 1024)

/* Pines cámara XIAO ESP32-S3 Sense */
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

static uint8_t *tensor_arena = nullptr;

static const char *tensor_type_to_str(TfLiteType type)
{
    switch (type) {
        case kTfLiteFloat32: return "float32";
        case kTfLiteInt8:    return "int8";
        case kTfLiteUInt8:   return "uint8";
        case kTfLiteInt16:   return "int16";
        case kTfLiteInt32:   return "int32";
        default:             return "otro";
    }
}

static void print_tensor_info(const char *name, const TfLiteTensor *tensor)
{
    if (tensor == nullptr) {
        ESP_LOGW(TAG_MODEL, "%s es NULL", name);
        return;
    }

    char dims_str[128];
    int offset = 0;

    offset += snprintf(dims_str + offset, sizeof(dims_str) - offset, "[");

    for (int i = 0; i < tensor->dims->size; i++) {
        offset += snprintf(
            dims_str + offset,
            sizeof(dims_str) - offset,
            "%d%s",
            tensor->dims->data[i],
            (i == tensor->dims->size - 1) ? "" : ", "
        );
    }

    snprintf(dims_str + offset, sizeof(dims_str) - offset, "]");

    ESP_LOGI(TAG_MODEL,
             "%s -> tipo=%s | bytes=%d | dims=%s | scale=%f | zero_point=%ld",
             name,
             tensor_type_to_str(tensor->type),
             tensor->bytes,
             dims_str,
             tensor->params.scale,
             (long)tensor->params.zero_point);
}

static esp_err_t camera_init_once(void)
{
    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QQVGA;   // 160x120
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(300));

    esp_err_t err = ESP_FAIL;

    for (int intento = 1; intento <= 5; intento++) {
        ESP_LOGI(TAG_MODEL, "Inicializando cámara, intento %d/5...", intento);

        err = esp_camera_init(&config);

        if (err == ESP_OK) {
            ESP_LOGI(TAG_MODEL, "Cámara inicializada correctamente");
            return ESP_OK;
        }

        ESP_LOGW(TAG_MODEL,
                 "Error inicializando cámara en intento %d: %s",
                 intento,
                 esp_err_to_name(err));

        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG_MODEL, "No se pudo inicializar la cámara tras varios intentos");
    return err;
}

static int8_t quantize_input_pixel(uint8_t pixel, TfLiteTensor *input)
{
    float normalized = ((float)pixel) / 255.0f;
    int q = (int)roundf(normalized / input->params.scale) + input->params.zero_point;

    if (q < -128) q = -128;
    if (q > 127) q = 127;

    return (int8_t)q;
}

static float dequantize_output_value(int8_t value, TfLiteTensor *output)
{
    return output->params.scale * ((int)value - output->params.zero_point);
}

/*
 * Convierte RGB565 a RGB888.
 * Estamos usando byte order: pixel = b1 << 8 | b0.
 */
static void rgb565_to_rgb888(uint8_t b0, uint8_t b1,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t pixel = ((uint16_t)b1 << 8) | b0;

    *r = (uint8_t)((((pixel >> 11) & 0x1F) * 255) / 31);
    *g = (uint8_t)((((pixel >> 5)  & 0x3F) * 255) / 63);
    *b = (uint8_t)((( pixel        & 0x1F) * 255) / 31);
}

static void print_average_color(camera_fb_t *fb)
{
    uint32_t sum_r = 0;
    uint32_t sum_g = 0;
    uint32_t sum_b = 0;
    int samples = 0;

    for (int y = 0; y < fb->height; y += 10) {
        for (int x = 0; x < fb->width; x += 10) {
            int idx = (y * fb->width + x) * 2;

            uint8_t b0 = fb->buf[idx];
            uint8_t b1 = fb->buf[idx + 1];

            uint8_t r, g, b;
            rgb565_to_rgb888(b0, b1, &r, &g, &b);

            sum_r += r;
            sum_g += g;
            sum_b += b;
            samples++;
        }
    }

    if (samples > 0) {
        ESP_LOGI(TAG_MODEL,
                 "Color medio aproximado de la foto: R=%u G=%u B=%u",
                 (unsigned int)(sum_r / samples),
                 (unsigned int)(sum_g / samples),
                 (unsigned int)(sum_b / samples));
    }
}

static void dump_frame_as_jpeg_base64(camera_fb_t *fb)
{
    uint8_t *jpg_buf = nullptr;
    size_t jpg_len = 0;

    bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);

    if (!ok || jpg_buf == nullptr || jpg_len == 0) {
        ESP_LOGE(TAG_MODEL, "No se pudo convertir frame a JPEG");
        return;
    }

    size_t b64_len = ((jpg_len + 2) / 3) * 4 + 1;

    uint8_t *b64_buf = (uint8_t *)heap_caps_malloc(
        b64_len,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (b64_buf == nullptr) {
        ESP_LOGE(TAG_MODEL, "No se pudo reservar memoria para base64");
        free(jpg_buf);
        return;
    }

    size_t out_len = 0;
    int ret = mbedtls_base64_encode(
        b64_buf,
        b64_len,
        &out_len,
        jpg_buf,
        jpg_len
    );

    if (ret != 0) {
        ESP_LOGE(TAG_MODEL, "Error codificando JPEG a base64");
        heap_caps_free(b64_buf);
        free(jpg_buf);
        return;
    }

    ESP_LOGI(TAG_MODEL, "JPEG generado para depuración: %u bytes", (unsigned int)jpg_len);

    printf("\nCAMERA_JPEG_BASE64_BEGIN\n");

    for (size_t i = 0; i < out_len; i += 128) {
        size_t chunk = out_len - i;
        if (chunk > 128) {
            chunk = 128;
        }

        fwrite(b64_buf + i, 1, chunk, stdout);
        printf("\n");
    }

    printf("CAMERA_JPEG_BASE64_END\n\n");

    heap_caps_free(b64_buf);
    free(jpg_buf);
}

static void fill_input_from_camera_frame(TfLiteTensor *input, camera_fb_t *fb)
{
    int8_t *dst = input->data.int8;

    const int out_w = 160;
    const int out_h = 160;

    const int src_w = fb->width;   // 160
    const int src_h = fb->height;  // 120

    const int y_offset = (out_h - src_h) / 2;  // 20

    int8_t pad = quantize_input_pixel(114, input);

    for (int i = 0; i < input->bytes; i++) {
        dst[i] = pad;
    }

    for (int y = 0; y < src_h; y++) {
        for (int x = 0; x < src_w; x++) {
            int src_index = (y * src_w + x) * 2;

            uint8_t b0 = fb->buf[src_index];
            uint8_t b1 = fb->buf[src_index + 1];

            uint8_t r, g, b;
            rgb565_to_rgb888(b0, b1, &r, &g, &b);

            int dst_y = y + y_offset;
            int dst_index = (dst_y * out_w + x) * 3;

            dst[dst_index + 0] = quantize_input_pixel(r, input);
            dst[dst_index + 1] = quantize_input_pixel(g, input);
            dst[dst_index + 2] = quantize_input_pixel(b, input);
        }
    }

    ESP_LOGI(TAG_MODEL, "Imagen de cámara copiada al input del modelo con letterbox 160x160");
}

static void print_best_detection(TfLiteTensor *output)
{
    const char *classes[6] = {
        "Cargo",
        "Carrier",
        "Cruise",
        "Sub",
        "Surface",
        "Tug"
    };

    int channels = output->dims->data[1];  // 10
    int preds = output->dims->data[2];     // 525

    ESP_LOGI(TAG_MODEL, "Output YOLO: channels=%d preds=%d", channels, preds);

    int8_t *out = output->data.int8;

    float best_score = -999.0f;
    int best_pred = -1;
    int best_class = -1;

    for (int p = 0; p < preds; p++) {
        for (int c = 0; c < 6; c++) {
            int channel = 4 + c;
            int index = channel * preds + p;

            float score = dequantize_output_value(out[index], output);

            if (score > best_score) {
                best_score = score;
                best_pred = p;
                best_class = c;
            }
        }
    }

    if (best_class < 0 || best_class >= 6) {
        ESP_LOGE(TAG_MODEL, "Clase inválida en salida");
        return;
    }

    ESP_LOGI(TAG_MODEL,
             "Mejor candidato: clase=%s score=%.3f pred=%d",
             classes[best_class],
             best_score,
             best_pred);

    if (best_score > 0.20f) {
        ESP_LOGI(TAG_MODEL, "POSIBLE BARCO DETECTADO: %s score=%.3f",
                 classes[best_class],
                 best_score);
    } else {
        ESP_LOGW(TAG_MODEL, "No hay detección clara. Mejor score=%.3f", best_score);
    }
}

extern "C" void test_barcos_model_load(void)
{
    ESP_LOGI(TAG_MODEL, "Memoria libre interna: %u bytes",
             (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    ESP_LOGI(TAG_MODEL, "Memoria libre PSRAM: %u bytes",
             (unsigned int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    const tflite::Model *model = tflite::GetModel(barcos_model_tflite);

    if (model == nullptr) {
        ESP_LOGE(TAG_MODEL, "No se pudo cargar el modelo TFLite");
        return;
    }

    ESP_LOGI(TAG_MODEL, "Modelo barcos cargado");
    ESP_LOGI(TAG_MODEL, "Tamaño modelo: %u bytes", barcos_model_tflite_len);
    ESP_LOGI(TAG_MODEL, "Schema version modelo: %d", (int)model->version());

    tensor_arena = (uint8_t *)heap_caps_malloc(
        TENSOR_ARENA_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (tensor_arena == nullptr) {
        ESP_LOGE(TAG_MODEL, "No se pudo reservar tensor_arena en PSRAM (%d bytes)",
                 TENSOR_ARENA_SIZE);
        return;
    }

    ESP_LOGI(TAG_MODEL, "Tensor arena reservado en PSRAM: %d bytes", TENSOR_ARENA_SIZE);

    static tflite::MicroMutableOpResolver<30> resolver;

    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddFullyConnected();
    resolver.AddReshape();
    resolver.AddTranspose();
    resolver.AddConcatenation();
    resolver.AddAdd();
    resolver.AddSub();
    resolver.AddMul();
    resolver.AddLogistic();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddMaxPool2D();
    resolver.AddPad();
    resolver.AddStridedSlice();
    resolver.AddResizeNearestNeighbor();
    resolver.AddMean();
    resolver.AddSplit();
    resolver.AddSplitV();

    static tflite::MicroInterpreter interpreter(
        model,
        resolver,
        tensor_arena,
        TENSOR_ARENA_SIZE
    );

    TfLiteStatus allocate_status = interpreter.AllocateTensors();

    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG_MODEL, "AllocateTensors() fallo");
        ESP_LOGE(TAG_MODEL, "Puede faltar una operacion en el resolver o memoria");
        return;
    }

    ESP_LOGI(TAG_MODEL, "AllocateTensors() correcto");

    ESP_LOGI(TAG_MODEL, "Numero de inputs: %d", (int)interpreter.inputs_size());
    for (size_t i = 0; i < interpreter.inputs_size(); i++) {
        char name[32];
        snprintf(name, sizeof(name), "input[%u]", (unsigned int)i);
        print_tensor_info(name, interpreter.input(i));
    }

    ESP_LOGI(TAG_MODEL, "Numero de outputs: %d", (int)interpreter.outputs_size());
    for (size_t i = 0; i < interpreter.outputs_size(); i++) {
        char name[32];
        snprintf(name, sizeof(name), "output[%u]", (unsigned int)i);
        print_tensor_info(name, interpreter.output(i));
    }

    if (camera_init_once() != ESP_OK) {
        ESP_LOGE(TAG_MODEL, "No se puede probar detección porque la cámara no inicia");
        return;
    }

    ESP_LOGI(TAG_MODEL, "Coloca la imagen delante de la cámara. Captura en 5 segundos...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == nullptr) {
        ESP_LOGE(TAG_MODEL, "No se pudo capturar imagen");
        return;
    }

    ESP_LOGI(TAG_MODEL, "FOTO CAPTURADA");

    ESP_LOGI(TAG_MODEL,
             "Frame capturado: width=%d height=%d len=%u format=%d",
             fb->width,
             fb->height,
             (unsigned int)fb->len,
             fb->format);

    print_average_color(fb);

    /*
     * Esto imprime la foto capturada en base64 para verla en el PC.
     * Cuando ya hayas comprobado la imagen, puedes comentar esta línea.
     */
    dump_frame_as_jpeg_base64(fb);

    TfLiteTensor *input = interpreter.input(0);

    fill_input_from_camera_frame(input, fb);

    esp_camera_fb_return(fb);

    ESP_LOGI(TAG_MODEL, "Ejecutando Invoke() con imagen real de cámara...");

    int64_t t0 = esp_timer_get_time();

    TfLiteStatus invoke_status = interpreter.Invoke();

    int64_t t1 = esp_timer_get_time();
    float infer_ms = (float)(t1 - t0) / 1000.0f;

    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG_MODEL, "Invoke() fallo");
        return;
    }

    ESP_LOGI(TAG_MODEL, "Invoke() correcto");
    ESP_LOGI(TAG_MODEL, "Tiempo de inferencia: %.2f ms", infer_ms);

    TfLiteTensor *output = interpreter.output(0);

    print_best_detection(output);

    int8_t *out = output->data.int8;

    ESP_LOGI(TAG_MODEL, "Primeros valores output int8:");
    for (int i = 0; i < 20 && i < output->bytes; i++) {
        ESP_LOGI(TAG_MODEL, "out[%d] = %d", i, out[i]);
    }

    ESP_LOGI(TAG_MODEL, "Prueba de camara + inferencia terminada correctamente");
}
