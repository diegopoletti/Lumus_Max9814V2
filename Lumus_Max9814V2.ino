//* @version "v2.1"
/* Includes ---------------------------------------------------------------- */
#include <Lumos_inferencing.h>  // Incluye la biblioteca de inferencia Lumos
#include "freertos/FreeRTOS.h"  // Incluye las bibliotecas FreeRTOS necesarias
#include "freertos/task.h"
#include "driver/adc.h"         // Incluye la biblioteca para el control del ADC
#include "esp_adc_cal.h"        // Incluye la biblioteca para la calibración del ADC
#include "driver/i2s.h"         // Incluye la biblioteca para el control del I2S

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;           // Buffer para almacenar muestras de audio
    uint8_t buf_ready;         // Indicador de buffer listo para inferencia
    uint32_t buf_count;        // Contador de muestras en el buffer
    uint32_t n_samples;        // Número de muestras por ventana de inferencia
} inference_t;

static inference_t inference;  // Instancia de la estructura de inferencia
static const uint32_t sample_buffer_size = 2048;  // Tamaño del buffer de muestras
static signed short sampleBuffer[sample_buffer_size];  // Buffer para almacenar las muestras de audio
static bool debug_nn = false;  // Cambiar a true para imprimir características generadas
static bool record_status = true;  // Estado de grabación de muestras de audio

/**
 * @brief Configuración del ADC e I2S
 */
void adc_init() {
    // Configurar el ADC1 en el canal 6 (GPIO34) con 12 bits de precisión y atenuación de 12 dB
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

    // Configuración del I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 16000,  // Tasa de muestreo de 16 kHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false
    };

    // Configurar I2S
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);
    i2s_adc_enable(I2S_NUM_0);
}

/**
 * @brief Configuración inicial de Arduino
 */
void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Configurar ADC e I2S
    adc_init();

    // Imprimir resumen de configuraciones de inferencia
    ei_printf("Configuraciones de inferencia:\n");
    ei_printf("\tIntervalo: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tTamaño del cuadro: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tDuración de la muestra: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNúmero de clases: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    ei_printf("\nIniciando inferencia continua en 2 segundos...\n");
    ei_sleep(2000);

    // Iniciar la captura de muestras de audio
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERROR: No se pudo asignar el búfer de audio (tamaño %d), esto podría deberse a la longitud de la ventana de su modelo\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    ei_printf("Grabando...\n");
}

/**
 * @brief se presenta una implementación para realizar la clasificación de audio utilizando Edge Impulse en un dispositivo Arduino. El código se encarga de grabar   audio a través de un micrófono, ejecutar el clasificador de Edge Impulse y mostrar las predicciones resultantes.
 * @note Conceptos Clave
    Microphone Inference Record: Función para grabar audio a través del micrófono.
    Run Classifier: Función que ejecuta el clasificador de Edge Impulse.
    Impresión de Resultados: Muestra las predicciones del clasificador y el puntaje de anomalía si está disponible.
    Activación de Pin: Activa o desactiva un pin GPIO según el resultado de la clasificación.
    Estructura del Código
    El código comienza grabando audio a través de la función microphone_inference_record(). Luego, se crea una señal de audio y se ejecuta el clasificador de Edge Impulse con esta señal. Posteriormente, se imprimen las predicciones del clasificador y se activa o desactiva un pin GPIO según el resultado de la clasificación. Finalmente, se resetea el watchdog en cada ciclo de loop.
 */
void loop() {
    bool m = microphone_inference_record(); // Realizar grabación de audio

    if (!m) {
        ei_printf("ERROR: Falló la grabación de audio...\n"); // Imprimir mensaje de error si la grabación falla
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;

    ei_impulse_result_t result = { 0 };

    // Ejecutar el clasificador de Edge Impulse
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERROR: Falló la ejecución del clasificador (%d)\n", r); // Imprimir mensaje de error si la ejecución del clasificador falla
        return;
    }

    // Imprimir las predicciones del clasificador
    ei_printf("Predicciones ");
    ei_printf("(DSP: %d ms., Clasificación: %d ms., Anomalía: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    // Comentar el bucle de predicciones del clasificador (comentado por razones de depuración)
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    puntaje de anomalía: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif

    digitalWrite(33, LOW);   // Desactivar el pin 33 (GPIO33)

    // Activa el pin 33 si la inferencia es mayor que 0.40000
    if (result.classification[0].value > 0.40000) {
        ei_printf("    %s: ", result.classification[0].label);
        ei_printf_float(result.classification[0].value);
        digitalWrite(33, HIGH);  // Activar el pin 33 (GPIO33)
    }

    // Resetear el watchdog en cada ciclo de loop
    vTaskDelay(pdMS_TO_TICKS(500));;
}

/**
 * @brief se presenta una función de callback para la inferencia de audio en un entorno de desarrollo Arduino. La función se encarga de procesar las muestras de audio recibidas y preparar el búfer para la inferencia.
  @note Conceptos Clave
  Callback: Función que se llama en respuesta a un evento específico, en este caso, la llegada de muestras de audio.
  Búfer de Inferencia: Almacena las muestras de audio necesarias para realizar la inferencia.
  n_samples: Número de muestras de audio a procesar en cada iteración.
  Estructura del Código
  La función audio_inference_callback recibe un parámetro n_samples, que representa la cantidad de muestras de audio a procesar. Luego, itera sobre cada muestra recibida y la agrega al búfer de inferencia. Cuando se alcanza la cantidad necesaria de muestras para la inferencia, se reinicia el contador del búfer y se marca como listo para la inferencia.
 */
static void audio_inference_callback(uint32_t n_samples) {
    for(int i = 0; i < n_samples; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i]; // Se añade cada muestra al búfer de inferencia

        if(inference.buf_count >= inference.n_samples) { // Si se alcanza el número de muestras necesario para la inferencia
          inference.buf_count = 0; // Se reinicia el contador del búfer
          inference.buf_ready = 1; // Se indica que el búfer está listo para la inferencia
        }
    }
}

/**
 * @brief Tarea para capturar muestras de audio del ADCEn el fragmento de código proporcionado, se presenta una función en Arduino c++ que se encarga de capturar muestras de audio de un canal específico de un convertidor analógico-digital (ADC). La función opera de manera continua mientras el estado de grabación esté activo, leyendo un número definido de muestras y realizando una llamada a una función de inferencia de audio.
 * @note Conceptos Clave
      samples_to_read: Cantidad de muestras a leer, definida como el argumento recibido por la función.
      sampleBuffer: Arreglo donde se almacenan las muestras de audio.
      adc1_get_raw(ADC1_CHANNEL_6): Función que obtiene la muestra del canal ADC1_CHANNEL_6.
      audio_inference_callback(samples_to_read): Función de inferencia de audio que se llama después de leer las muestras.
      vTaskDelay(pdMS_TO_TICKS(5)): Pausa de 5 milisegundos entre lecturas de muestras.
      vTaskDelete(NULL): Eliminación de la tarea actual al finalizar la captura de muestras.
    Estructura del Código
      La función capture_samples recibe como argumento la cantidad de muestras a leer. Dentro de un bucle while, se lee el número especificado de muestras del canal ADC1_CHANNEL_6 y se almacenan en sampleBuffer. Luego, se verifica el estado de grabación y se llama a la función de inferencia de audio si la grabación está activa. Finalmente, se realiza una pausa de 10 milisegundos antes de repetir el proceso. Una vez que el estado de grabación se desactiva, la tarea se elimina.  
 */
static void capture_samples(void* arg) {
    const uint32_t samples_to_read = (uint32_t)arg;
    size_t bytes_read;

    while (record_status) {
        // Leer datos del I2S
        i2s_read(I2S_NUM_0, (void*)sampleBuffer, samples_to_read * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        if (record_status) {
            // Llamar a la función de callback con el número de muestras leídas
            audio_inference_callback(samples_to_read);
        } else {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

/**
 * @brief Inicializa la estructura de inferencia y configura/inicia el ADC
 *
 * @param[in] n_samples El número de muestras
 *
 * @return Verdadero si la inicialización fue exitosa, falso de lo contrario
 */
static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    record_status = true;

    // Crear tarea para capturar muestras de audio
    xTaskCreate(capture_samples, "CapturarMuestras", 1024 * 32, (void*)sample_buffer_size, 10, NULL);

    return true;
}

/**
 * @brief Espera nuevos datos
 *
 * @return Verdadero cuando finaliza
 */
static bool microphone_inference_record(void) {
    bool ret = true;

    while (inference.buf_ready == 0) {
        delay(5);
    }

    inference.buf_ready = 0;
    return ret;
}

/**
 * @brief se presenta una función llamada microphone_audio_signal_get_data que se encarga de convertir muestras de señal de audio de tipo int16 a float. Esta conversión es esencial cuando se trabaja con datos de audio en diferentes formatos dentro de un proyecto de Arduino.
  @note Conceptos Clave
  Conversión de datos de audio de int16 a float.
  Uso de punteros para manipular datos en Arduino.
  Retorno de un valor para indicar el éxito de la operación.
  Estructura del Código
  La función microphone_audio_signal_get_data toma tres parámetros: offset (desplazamiento inicial en el buffer de entrada), length (longitud de datos a procesar) y out_ptr (puntero al buffer de salida donde se almacenarán los datos convertidos).
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

/**
 * @brief Detiene la captura de audio y libera los buffers
 */
static void microphone_inference_end(void) {
    record_status = false;
    ei_free(inference.buffer);
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
}
