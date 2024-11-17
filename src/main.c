/**
 * En este código la idea es realizar la lectura de datos por medio de dos
 * canales del ADC, el canal 0 y 1. Por el canal 0 sensamos la temperatura
 * utilizando el DHT11. Por el canal 1 sensamos el estado de la batería por
 * medio de un divisor resistivo. Utilizando la LCD 16x2 conectada, se indica la
 * temperatura, si es modo manual o automático y el estado de la batería. Y los
 * datos que se capturan serán enviados por UART, y se los gráficara para ver su
 * variación a lo largo del tiempo.
 *
 * Lo comentado anteriormente se esta cumpliendo todo menos los datos obtenidos
 * del segundo canal, por algún motivo no se logra una independencia completa
 * entre los canales del ADC. El canal 0 fácilmente recorre los valores de 0 a
 * 4096, pero el canal 1 como que toma valores +-100 de los que toma el canal 0.
 *
 * Un led indica el estado de la batería, cambiará su brillo por medio de la
 * señal PWM. (A menor brillo batería más descargada). Otro led es el de la
 * alarma que indica si la temperatura es peligrosa o hay una detección de
 * movimiento.
 *
 * La señal PWM sale por el pin B9. Es decir, el led que según su brillo indica
 * el estado de la batería ya estaría configurado. Cuando la temperatura supera
 * cierto umbral se enciende el led pc13.
 *
 *
 *
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> // Incluir para usar strlen

#include "lcd_i2c.h"

#define STACK_SIZE configMINIMAL_STACK_SIZE
#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY + 2)
#define tskLOW_PRIORITY_ADC ((UBaseType_t)tskIDLE_PRIORITY + 3)
#define MAX_COUNT 10000    // Máximo valor del PWM
#define ADC_MAX_VALUE 4095 // Valor máximo de 12 bits para el ADC
#define UMBRAL_TEMP_C 20   // Umbral de temperatura en °C

// Definiciones de pins
// TO DO: Agregar definiciones de los pines

// variables Globales
volatile float temperatura_C = 0, bateria_porcetaje = 0;
volatile uint16_t temperatura = 0, porcentajeBateria = 0; // Valores RAW
uint8_t modo_sistema = 0;
uint16_t sensor_movimiento = 0;

// SEMAFOROS
xSemaphoreHandle temp_semaforo = 0;
xSemaphoreHandle bateria_semaforo = 0;
xSemaphoreHandle movimiento_semaforo = 0;

// rutina para controlar el stack overflow
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)),
                                   char *pcTaskName __attribute__((unused))) {
  while (1) {
  }
}

void inicializar_semaforos() {
  temp_semaforo = xSemaphoreCreateMutex();
  bateria_semaforo = xSemaphoreCreateMutex();
  movimiento_semaforo = xSemaphoreCreateMutex();
}

void configure_pins() {
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);

  // TODO: Agregar configuración de los pines
}

void configure_usart(void) {
  rcc_periph_clock_enable(RCC_USART1); // Habilita USART1
  rcc_periph_clock_enable(RCC_GPIOA);  // Habilita GPIOA para USART
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX); // Configura PA9 como TX
  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
}

void usart_send_labeled_value(const char *label, uint16_t value) {
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label,
                     value); // Formato: "Etiqueta:Valor"
  for (int i = 0; i < len; i++) {
    usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
  }
}

void configure_adc(void) {
  // Habilita el reloj para el ADC1
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_GPIOA);

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);

  // Apaga el ADC para configurarlo
  adc_power_off(ADC1);

  // Configuro el tiempo de muestreo
  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_55DOT5CYC);
  adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_55DOT5CYC);
  // Selecciono el canal 0
  // adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // ?
  // Prendo el ADC
  adc_power_on(ADC1);
  // Calibro el ADC
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

void configure_pwm(void) {
  /* Configuración del GPIOB y los pines */
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOB,                          // Puerto correspondiente
                GPIO_MODE_OUTPUT_2_MHZ,         // Máxima velocidad de switcheo
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, // Función alternativa
                GPIO9);                         // PB9 para el canal de PWM

  /* Configuración del TIM4 para PWM centrado */
  rcc_periph_clock_enable(RCC_TIM4);
  timer_set_mode(TIM4,                 // Timer general 4
                 TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
                 TIM_CR1_CMS_CENTER_1, // Modo centrado
                 TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

  timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz

  // Configura el canal PWM para el LED en PB7
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1); // PWM1: activo alto
  timer_enable_oc_output(TIM4, TIM_OC4);          // Habilitar salida OC2

  // Activa el contador del timer
  timer_enable_counter(TIM4);
}

// // asi se crean las tareas
// static void task1(void *args __attribute__((unused)))
// {
//     while (true)
//     {
//         gpio_toggle(GPIOC, GPIO13);
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

static void task_i2c(void *args __attribute__((unused))) {
  while (true) {
    char buffer_temp_bateria[80];
    char buffer_modo[20];

    // Usar sprintf para formatear los valores flotantes
    sprintf(buffer_temp_bateria, "Temp: %d Bat: %d", (int)temperatura_C, (int)porcentajeBateria);  // Convertir la temperatura a cadena

   sprintf(buffer_modo, "Modo: %s", (modo_sistema == 0) ? "Auto" : "Manual");  // Convertir el modo a cadena

    lcd_set_cursor(0, 0);
    lcd_print(buffer_temp_bateria);
    lcd_set_cursor(1, 0);
    lcd_print(buffer_modo);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void task_uart(void *args __attribute__((unused))) {
  while (true) {
    // Enviar la temperatura con un decimal
    usart_send_labeled_value(">A1: %.1f\n", temperatura_C);

    // Enviar el porcentaje de batería con un decimal
    usart_send_labeled_value(">A2: %.1f\n", bateria_porcetaje);

    // Enviar el valor del sensor de movimiento (por ejemplo, como entero)
    usart_send_labeled_value(">V3: %d\n", sensor_movimiento);

    vTaskDelay(
        pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de enviar nuevamente
  }
}

static void task_adc(void *args __attribute__((unused))) {
  while (true) {
    adc_disable_scan_mode(
        ADC1); // Deshabilitar modo escaneo para una conversión única
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1))
      ;
    temperatura = adc_read_regular(ADC1);
    temperatura_C = (float)temperatura * (3.3f / 4095.0f) * 100.0f;

    adc_disable_scan_mode(
        ADC1); // Asegurar que no queden configuraciones residuales
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL1);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1))
      ;
    porcentajeBateria = adc_read_regular(ADC1);
    bateria_porcetaje = (float)porcentajeBateria * (100.0f / 4095.0f);

    if (temperatura_C < UMBRAL_TEMP_C)
      gpio_set(GPIOC, GPIO13);
    else
      gpio_clear(GPIOC, GPIO13);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Tarea PWM
static void task_pwm(void *args __attribute__((unused))) {
  uint16_t pwm_val = 0;

  while (true) {
    // Escala el valor del ADC al rango del PWM
    pwm_val = (temperatura * MAX_COUNT) / ADC_MAX_VALUE;

    // Ajusta el ciclo de trabajo del PWM en función del valor del ADC
    timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
  }
}

int main(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  configure_pins();
  configure_usart();
  configure_adc();
  configure_pwm();
  lcd_init();

  // ENCENDEMOS el led
  gpio_set(GPIOC, GPIO13);

  // xSemaphoreHandle semaforo_dma = xSemaphoreCreateMutex(); // ?

  // creamos las tareas
  // xTaskCreate(task1, "LedSwitching", configMINIMAL_STACK_SIZE, NULL,
  //          tskLOW_PRIORITY, NULL);

  xTaskCreate(task_i2c, "I2C", configMINIMAL_STACK_SIZE, NULL, tskLOW_PRIORITY,
              NULL);

  xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL,
              tskLOW_PRIORITY, NULL);

  // creamos la tarea ADC
  xTaskCreate(task_adc, "ADC", STACK_SIZE, NULL, tskLOW_PRIORITY_ADC, NULL);

  // creamos la tarea PWM
  xTaskCreate(task_pwm, "PWM", STACK_SIZE, NULL, tskLOW_PRIORITY, NULL);

  // iniciamos todas las tareas
  vTaskStartScheduler();

  while (1) {
  }
}
