// -------------------------- Librerías --------------------------------
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <stdio.h>
#include <task.h>

#include "lcd_i2c.h"

// ------------------------- Definiciones ------------------------------
// Definiciones para el timer
#define TIMER_PRESCALER 7199     // prescaler para 10khz
#define TIMER_PERIOD_ADC 9999    // periodo para 1khz
#define TIMER_PERIOD_UART 299999 // periodo para 30 seg
#define BATERY_SENSE_TIME 15     // 1Leeremos la bateria cada 10 seg;

// Definiciones para el USART
#define USART_BAUDRATE 9600
#define USART_DATABITS 8

// BOLEANOS
#define TRUE 1
#define FALSE 0

// Definiciones EXTI
#define FALLING 0
#define RISING 1

// magic numbers
#define BUFFER_SIZE 16
#define MAX_TEMP 50
#define MIN_TEMP -10
#define MAX_COUNT 10000    // Máximo valor del PWM
#define ADC_MAX_VALUE 4095 // Valor máximo de 12 bits para el ADC
// ------------------------- Variables   ------------------------------
// Create a binary semaphore
xSemaphoreHandle adc_dma_semaphore = NULL;
xSemaphoreHandle control_semaphore = NULL;
xSemaphoreHandle i2c_semaphore = NULL;
xSemaphoreHandle uart_semaphore = NULL;
xSemaphoreHandle alarma_semaphore = NULL;

// variables Globales
volatile uint16_t temperatura = 0, porcentajeBateria = 0;
volatile float temperatura_C = 0, bateria_porcetaje = 0;
bool modo_sistema = 0; // 0: manual, 1: automatico
uint16_t sensor_movimiento = 0;
volatile uint16_t Timer_Batery_Count = 0; // contador para leer la bateria

char buffer[BUFFER_SIZE];

// SEMAFOROS
xSemaphoreHandle temp_semaforo = 0;
xSemaphoreHandle bateria_semaforo = 0;
xSemaphoreHandle movimiento_semaforo = 0;

char buffer_temp_bateria[20];
char buffer_modo[24];

// ------------------------- Configuración
// ------------------------------------------------
/**
 * Función para configurar los pines.
 */
void configure_pins() {
  rcc_periph_clock_enable(RCC_GPIOA); // Habilita GPIOA para USART
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX); // Configura PA9 como TX
  // configuracion pin para Boton de Modo
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);
  // configuracion pin para sensor movimiento
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);

  rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO8); // Configurar B8 como salida
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO9); // PB9 para el canal de PWM

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);

  // TODO: Agregar configuración de los pines
}

/**
 * Función para configurar el UART.
 */
void configure_usart(void) {
  rcc_periph_clock_enable(RCC_USART1); // Habilita USART1
  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
}

/**
 * Función para configurar el TIMER.
 */
void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2); // Enable TIM2 clock
  timer_disable_counter(TIM2);
  // timer_reset(TIM2); // Reset TIM2 configuration
  timer_set_prescaler(TIM2, 7200 - 1); // Prescaler for 10 kHz timer clock
  timer_set_period(TIM2, 200 - 1);     // Period for 1-second interrupt
  // timer_enable_update_event(TIM2);    // Enable update events
  timer_enable_irq(TIM2, TIM_DIER_UIE); // Enable update interrupt
  nvic_enable_irq(NVIC_TIM2_IRQ);       // Enable TIM2 interrupt in NVIC
  timer_enable_counter(TIM2);           // Start the timer

  rcc_periph_clock_enable(RCC_TIM3);   // Enable Timer 3 clock
  timer_disable_counter(TIM3);         // Ensure timer is stopped during setup
  timer_set_prescaler(TIM3, 7200 - 1); // Prescaler for 10 kHz clock
  timer_set_period(TIM3,
                   300000 - 1); // Period for 30 seconds (10,000 Hz / 300,000)
  timer_enable_irq(TIM3, TIM_DIER_UIE); // Enable update interrupt
  nvic_enable_irq(NVIC_TIM3_IRQ);       // Enable NVIC interrupt for Timer 3
  timer_enable_counter(TIM3);           // Start the timer
}

/**
 * Función para configurar el ADC.
 */
void configure_adc(void) {
  rcc_periph_clock_enable(RCC_ADC1); // Habilita el reloj para el ADC1

  adc_power_off(ADC1); // Apaga el ADC para configurarlo

  // Configura el tiempo de muestreo para cada canal

  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); // Canal 0
  adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_1DOT5CYC); // Canal 1
  // Configura la secuencia regular de canales
  adc_set_regular_sequence(
      ADC1, 1, (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1  /

  // Habilita la interrupción de fin de conversión (EOC) del ADC
  adc_enable_eoc_interrupt(ADC1);
  // Habilita la interrupción del ADC en el NVIC
  nvic_enable_irq(NVIC_ADC1_2_IRQ);

  // Configura el ADC para que dispare la transferencia de datos
  adc_enable_dma(ADC1); // Habilita DMA (si es necesario)

  adc_power_on(ADC1); // Enciende el ADC

  adc_reset_calibration(ADC1); // Reinicia la calibración
  adc_calibrate(ADC1);         // Calibra el ADC
}

/**
 * Función para configurar el DMA.
 */
void configure_dma(void) {
  // Habilita el reloj para el DMA1 y GPIOC
  rcc_periph_clock_enable(RCC_DMA1);

  // Configura el dma para transferir datos
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  // Origen de los datos a transferir:
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, &ADC_DR(ADC1));
  // Dirección de datos destino. El ODR (Output Data Register) del GPIOA:
  dma_set_memory_address(DMA1, DMA_CHANNEL1, &porcentajeBateria);
  // Tamaño del dato a leer
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
  // Tamaño del dato a escribir
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

  // Cantidad de datos a transferir:
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);

  // Se incrementa automaticamente la posición en memoria:
  dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1);
  // La dirección destino se mantiene fija:
  dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
  // Se establece la prioridad del canal 7 del DMA1 como alta:
  dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
  // Se habilita el modo circular para que la transferencia se repita
  // indefinidamente
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  // Se habilita la interrupción que se ejecutan al finalizar la transferencia
  // para togglear un pin (no es necesario)
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

  // Se habilita en el NVIC la interrupción del DMA1-CH7
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
}

/**
 * Configuración de la señal PWM, utilizando el timer 4.
 */
void configure_pwm(void) {

  /* Configuración del TIM4 para PWM centrado */
  rcc_periph_clock_enable(RCC_TIM4);
  timer_set_mode(TIM4,                 // Timer general 4
                 TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
                 TIM_CR1_CMS_CENTER_1, // Modo centrado
                 TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

  timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz

  // Configura el canal PWM para el LED en PB7
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1); // PWM1: activo alto
  timer_enable_oc_output(TIM4, TIM_OC4);          // Habilitar salida OC4

  // Activa el contador del timer
  timer_enable_counter(TIM4);
}

void configure_exti(void) {
  rcc_periph_clock_enable(RCC_AFIO); // Habilita el reloj para AFIO
  nvic_enable_irq(
      NVIC_EXTI4_IRQ); // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(
      EXTI4, GPIO4); // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(
      EXTI4, EXTI_TRIGGER_BOTH); // Configura el trigger de la interrupción
  exti_enable_request(EXTI4);    // Habilita la solicitud de interrupción

  nvic_enable_irq(
      NVIC_EXTI3_IRQ); // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(
      EXTI3, GPIO3); // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(
      EXTI3, EXTI_TRIGGER_BOTH); // Configura el trigger de la interrupción
  exti_enable_request(EXTI3);    // Habilita la solicitud de interrupción
}

//------------------- Tareas
//--------------------------------------------------------------------
/**
 * Tarea que permite la comunicación UART entre la placa y la computadora.
 */
static void task_uart(void *args __attribute__((unused))) {
  while (true) {
    // Envía las variables etiquetadas por UART
    if (xSemaphoreTake(uart_semaphore, portMAX_DELAY) == pdTRUE) {
      usart_send_labeled_value(
          "Bat",
          porcentajeBateria); // Envía el valor del ADC por el puerto serial
      usart_send_labeled_value(
          "Temp",
          temperatura); // Envía el valor del ADC por el puerto serial
      usart_send_labeled_value("Modo", modo_sistema);
      usart_send_labeled_value("Sensor", sensor_movimiento);
    }
  }
}

/**
 * Tarea que realiza la lectura de los puerto A0 Y A1 utilizando el adc y dma.
 */
static void task_adc_dma(void *args __attribute__((unused))) {
  while (true) {
    // Wait for the semaphore indefinitely
    if (xSemaphoreTake(adc_dma_semaphore, portMAX_DELAY) == pdTRUE) {
      Timer_Batery_Count++;
      adc_power_off(ADC1); // Apaga el ADC para configurarlo
      dma_disable_channel(DMA1,
                          DMA_CHANNEL1); // Deshabilita el canal 1 del DMA1

      // Verifica si el contador es mayor a 5
      if ((Timer_Batery_Count == BATERY_SENSE_TIME)) {
        Timer_Batery_Count = 0;
        adc_set_regular_sequence(
            ADC1, 1,
            (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1
        dma_set_memory_address(
            DMA1, DMA_CHANNEL1,
            &porcentajeBateria);                // Dirección de memoria destino
        adc_power_on(ADC1);                     // Enciende el ADC
        adc_start_conversion_direct(ADC1);      // Inicia la conversión
        dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
        Timer_Batery_Count = 0;
      } else {
        adc_set_regular_sequence(
            ADC1, 1,
            (uint8_t[]){ADC_CHANNEL0}); // Secuencia de 1 canal: canal 1
        dma_set_memory_address(DMA1, DMA_CHANNEL1,
                               &temperatura); // Dirección de memoria destino
        adc_power_on(ADC1);                   // Enciende el ADC
        adc_start_conversion_direct(ADC1);
        dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
      }
    }
  }
}

/**
 * Tarea que permite visualizar las variables de interes en la LCD 16x2.
 */
static void task_i2c(void *args __attribute__((unused))) {
  while (true) {
    if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE) {
      // modo_sistema = 0;
      // lcd_clear();
      sprintf(buffer_temp_bateria, "Temp:%3d Bat:%3d", (int)temperatura_C,
              (int)bateria_porcetaje);
      if (modo_sistema == 0) {
        sprintf(buffer_modo, "Modo: %-6s", "Auto"); //?
      } else {
        sprintf(buffer_modo, "Modo: %-6s", "Manual");
      }
      lcd_set_cursor(0, 0);
      lcd_print(buffer_temp_bateria);
      lcd_set_cursor(1, 0);
      lcd_print(buffer_modo);
    }
  }
}

static void task_control(void *args __attribute__((unused))) {
  while (true) {
    if (xSemaphoreTake(control_semaphore, portMAX_DELAY) == pdTRUE) {
      gpio_toggle(GPIOC, GPIO13);
      temperatura_C = ((float)temperatura / 4095.0f) * 80.0f - 20.0f;

      if (temperatura_C > 60.0f)
        temperatura_C = 60.0f;
      if (temperatura_C < -20.0f)
        temperatura_C = 0.0f;

      bateria_porcetaje = (float)porcentajeBateria * (100.0f / 4095.0f);

      // Limitar el valor del porcentaje de batería a un máximo de 100%
      if (bateria_porcetaje > 100.0f)
        bateria_porcetaje = 100.0f;
      if (bateria_porcetaje < 0.0f)
        bateria_porcetaje = 0.0f;

      xSemaphoreGive(i2c_semaphore);    // Give the semaphore from I2C
      xSemaphoreGive(alarma_semaphore); // Give the semaphore from alarma
    }
  }
}

/**
 * Tarea que se activa de acuerdo al semaforo: alarma_semaphore.
 * Si se supera MAX_TEMP o es menor a MIN_TEMP se envía una señal
 * al buzzer y se hace parpadear un led para indicar la situación.
 */
static void task_alarma(void *args __attribute__((unused))) {
  while (true) {
    if (xSemaphoreTake(alarma_semaphore, portMAX_DELAY) == pdTRUE) {
      while (temperatura_C > MAX_TEMP || temperatura_C < MIN_TEMP ||
             sensor_movimiento == TRUE) {
        gpio_set(GPIOB, GPIO8);         // Enciende el buzzer en B8
        vTaskDelay(pdMS_TO_TICKS(100)); // Sonido durante 500 ms (ajustable)
        gpio_clear(GPIOB, GPIO8);       // Apaga el buzzer
        vTaskDelay(pdMS_TO_TICKS(100)); // Sonido durante 500 ms (ajustable)

        if (gpio_get(GPIOA, GPIO3)) {
          sensor_movimiento = FALSE;
        }
      }
    }
  }
}

/**
 * Tarea que permite llevar a cabo la señal PWM. Esta señal es recibida por el
 * led para indicar el estado de la batería según su brillo.
 */
static void task_pwm(void *args __attribute__((unused))) {
  uint16_t pwm_val = 0;

  while (true) {
    // Escala el valor del ADC al rango del PWM
    pwm_val = (temperatura * MAX_COUNT) / ADC_MAX_VALUE;

    // Ajusta el ciclo de trabajo del PWM en función del valor del ADC
    timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
  }
}

// -------------------------- Otras funciones ----------------------------------
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  // Handle stack overflow
  (void)xTask;      // Avoid unused parameter warning
  (void)pcTaskName; // Avoid unused parameter warning

  // Infinite loop for debugging purposes
  while (1) {
  }
}

void usart_send_labeled_value(const char *label, uint16_t value) {
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label,
                     value); // Formato: "Etiqueta:Valor"
  for (int i = 0; i < len; i++) {
    usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
  }
}

/**
 * Función que permite inicializar los semaforos.
 * Tener en cuenta que se deben inicializar los semaforos antes de que las
 * funciones los vayan a utilizar.
 */
void inicializacion_semanforos(void) {
  i2c_semaphore = xSemaphoreCreateBinary();
  adc_dma_semaphore = xSemaphoreCreateBinary();
  control_semaphore = xSemaphoreCreateBinary();
  alarma_semaphore = xSemaphoreCreateBinary();
  uart_semaphore = xSemaphoreCreateBinary();

  if (i2c_semaphore == NULL || adc_dma_semaphore == NULL ||
      control_semaphore == NULL || alarma_semaphore == NULL ||
      uart_semaphore == NULL) {
    while (1)
      ; // Handle semaphore creation failure
  }
}

/**
 * Función que crea las distintas tareas a realizar.
 */
void tareas_por_hacer(void) {
  // Create tasks
  xTaskCreate(task_adc_dma, "LED Control", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_i2c, "I2C", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_control, "control", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_alarma, "alarma", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  // creamos la tarea PWM
  xTaskCreate(task_pwm, "PWM", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
}

// -------------------------- Interrupciones -------------------------------
/**
 * Manejo de la interrupción para el timer 2.
 */
void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) { // Check for update interrupt
    timer_clear_flag(TIM2, TIM_SR_UIF);   // Clear the interrupt flag
    xSemaphoreGiveFromISR(adc_dma_semaphore,
                          NULL); // Give the semaphore from ISR
  }
}

// Timer 3 ISR
void tim3_isr(void) {
  if (timer_get_flag(TIM3, TIM_SR_UIF)) { // Check update interrupt flag
    timer_clear_flag(TIM3, TIM_SR_UIF);   // Clear interrupt flag
    xSemaphoreGiveFromISR(uart_semaphore,
                          NULL); // Give the semaphore from ISR
  }
}

/**
 * Manejo de la interrupción para el adc, canal 0 y 1.
 */
void adc1_2_isr(void) {
  if (adc_eoc(ADC1)) {
    // Limpia el flag de fin de conversión (EOC)
    adc_clear_flag(ADC1, ADC_SR_EOC);
  }
}

/**
 * Manejo de la interrupción para el dma.
 */
void dma1_channel1_isr(void) {
  //     // Limpia el flag de transferencia completa
  dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CTCIF1);
  xSemaphoreGiveFromISR(control_semaphore, NULL); // Give the semaphore from ISR
}

/**
 * Manejo de interrupciónes externas para el switch que permite
 * cambiar de modo, entre automático y manual.
 */
void exti4_isr(void) {
  exti_reset_request(EXTI4); // Limpia la solicitud de interrupción

  modo_sistema = gpio_get(GPIOA, GPIO4);
}

/**
 * Manejo de interrupción externa para el sensor de moviemiento.
 */
void exti3_isr(void) {
  exti_reset_request(EXTI3); // Limpia la solicitud de interrupción
  sensor_movimiento = TRUE;
}

// -------------------------------- Funcion principal ---------------------
int main(void) {

  inicializacion_semanforos();

  configure_pins();
  configure_usart();
  configure_timer(); // Initialize timer // Cuidado con cambiar el orden!
  configure_adc();
  configure_dma();
  configure_pwm();
  configure_exti();
  lcd_init();

  adc_start_conversion_direct(ADC1);

  tareas_por_hacer();

  // Start the scheduler
  vTaskStartScheduler();

  // Infinite loop (should never reach here)
  while (1) {
  }

  return 0;
}
