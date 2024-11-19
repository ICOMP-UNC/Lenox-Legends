// -------------------------- Librerías --------------------------------
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include <FreeRTOS.h>
#include <semphr.h>
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

// magic numbers
#define BUFFER_SIZE 16
// ------------------------- Variables   ------------------------------
// Create a binary semaphore
xSemaphoreHandle led_semaphore = NULL;

// variables Globales
volatile uint16_t temperatura = 0;
volatile uint16_t porcentajeBateria = 0;
uint8_t modo_sistema = 0;
uint16_t sensor_movimiento = 0;

// variables globales
volatile uint16_t Timer_Batery_Count = 0; // contador para leer la bateria
volatile uint8_t REINICIO = 1;

char buffer[BUFFER_SIZE];

// SEMAFOROS
xSemaphoreHandle temp_semaforo = 0;
xSemaphoreHandle bateria_semaforo = 0;
xSemaphoreHandle movimiento_semaforo = 0;

char buffer_temp_bateria[32];
char buffer_modo[32];

// -------------------------------------- Configuración ------------------------------------------------
/**
 * Función para configurar los pines.
 */
void configure_pins() {
  rcc_periph_clock_enable(RCC_GPIOA); // Habilita GPIOA para USART
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX); // Configura PA9 como TX

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

  timer_enable_counter(TIM2); // Start the timer
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
  adc_set_regular_sequence(ADC1, 1,(uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1  /

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

//------------------- Tareas --------------------------------------------------------------------
/**
 * Tarea que permite la comunicación UART entre la placa y la computadora.
 */
static void task_uart(void *args __attribute__((unused))) {
  while (true) {
    // Envía las variables etiquetadas por UART
    usart_send_labeled_value("A1",
                             temperatura); // Envía adc_value1 con etiqueta "A1"
    usart_send_labeled_value(
        "A2", porcentajeBateria); // Envía adc_value2 con etiqueta "A2"
    usart_send_labeled_value(
        "V3", sensor_movimiento); // Envía variable3_uart con etiqueta "V3"
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Tarea que realiza la lectura de los puerto A0 Y A1 utilizando el adc y dma.
 */
static void task_adc_dma(void *args __attribute__((unused))) {
  while (true) {
    // Wait for the semaphore indefinitely
    if (xSemaphoreTake(led_semaphore, portMAX_DELAY) == pdTRUE) {
      gpio_toggle(GPIOC, GPIO13);
      adc_power_off(ADC1); // Apaga el ADC para configurarlo
      dma_disable_channel(DMA1,
                          DMA_CHANNEL1); // Deshabilita el canal 1 del DMA1

      // Verifica si el contador es mayor a 5
      if ((Timer_Batery_Count == BATERY_SENSE_TIME) || (REINICIO == 1)) {
        Timer_Batery_Count = 0;
        REINICIO = 0;
        adc_set_regular_sequence(
            ADC1, 1,
            (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1
        dma_set_memory_address(DMA1, DMA_CHANNEL1,
                               &porcentajeBateria);       // Dirección de memoria destino
        adc_power_on(ADC1);                     // Enciende el ADC
        adc_start_conversion_direct(ADC1);      // Inicia la conversión
        dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
        Timer_Batery_Count = 0;
      } else {
        adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL0}); // Secuencia de 1 canal: canal 1
        dma_set_memory_address(DMA1, DMA_CHANNEL1,&temperatura); // Dirección de memoria destino
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
    modo_sistema = 0;
    sprintf(buffer_temp_bateria, "Temperatura: %d", temperatura);
    if (modo_sistema == 0) {
      sprintf(buffer_modo, "Modo: Auto", modo_sistema);
    } else {
      sprintf(buffer_modo, "Modo: Manual", modo_sistema);
    }
    lcd_set_cursor(0, 0);
    lcd_print(buffer_temp_bateria);
    lcd_set_cursor(1, 0);
    lcd_print(buffer_modo);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ------------------------------------- Otras funciones
// --------------------------- Hook function for stack overflow
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

// --------------------------------------- Interrupciones -------------------------------
/**
 * Manejo de la interrupción para el timer 2.
 */
void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {       // Check for update interrupt
    timer_clear_flag(TIM2, TIM_SR_UIF);         // Clear the interrupt flag
    xSemaphoreGiveFromISR(led_semaphore, NULL); // Give the semaphore from ISR
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
  usart_send_labeled_value(
      "Bat:",
      porcentajeBateria); // Envía el valor del ADC por el puerto serial
  usart_send_labeled_value(
      "Temp:",
      temperatura); // Envía el valor del ADC por el puerto serial
}

// -------------------------------- Funcion principal ---------------------
int main(void) {
  configure_pins();
  configure_usart();
  configure_adc();
  configure_dma();

  lcd_init();

  // Create the binary semaphore
  led_semaphore = xSemaphoreCreateBinary();
  if (led_semaphore == NULL) {
    while (1)
      ; // Handle semaphore creation failure
  }

  configure_timer(); // Initialize timer // Cuidado con cambiar el orden!

  // Optionally give semaphore initially
  // xSemaphoreGive(led_semaphore);

  // Create tasks
  xTaskCreate(task_adc_dma, "LED Control", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(task_i2c, "I2C", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);

  // Start the scheduler
  vTaskStartScheduler();

  // Infinite loop (should never reach here)
  while (1) {
  }

  return 0;
}
