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
#include <libopencm3/stm32/dma.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> // Incluir para usar strlen

#define STACK_SIZE configMINIMAL_STACK_SIZE
#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY + 2)
#define tskLOW_PRIORITY_ADC ((UBaseType_t)tskIDLE_PRIORITY + 3)
#define ADC_MAX_VALUE 4095 // Valor máximo de 12 bits para el ADC
#define UMBRAL_TEMP_C 20   // Umbral de temperatura en °C


// variables Globales
volatile uint16_t temperatura = 0, porcentajeBateria = 0; // Valores RAW
volatile static uint16_t adc_values[2]; // Array para los valores del ADC

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
  //rcc_periph_clock_enable(RCC_GPIOA); // Habilitar el reloj para el puerto A
  //gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5); // A5 como entrada para el sensor de movimiento

  rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8); // Configurar B8 como salida
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11); // Configurar B11 como entrada flotante para el sensor de movimiento
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); // B13 como salida para el LED amarillo

  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
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

void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2);       // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, 7199);         // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, 9999);            // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE);    // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);          // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2);              // Inicia el contador del Timer 2
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

  adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // Configura la secuencia regular para un canal
  
  // Habilita DMA antes de encender el ADC
  adc_enable_dma(ADC1);

   // Habilita la interrupción de fin de conversión (EOC) del ADC
    adc_enable_eoc_interrupt(ADC1);
    nvic_enable_irq(NVIC_ADC1_2_IRQ); // Habilita la interrupción del ADC en el NVIC
  
  // Prendo el ADC
  adc_power_on(ADC1);
  // Calibro el ADC
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

static void task_uart(void *args __attribute__((unused))) {
  while (true) {
    // Enviar la temperatura con un decimal
    usart_send_labeled_value("A1", temperatura);

    // Enviar el porcentaje de batería con un decimal
    usart_send_labeled_value("A2", porcentajeBateria);

    vTaskDelay(
        pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de enviar nuevamente
  }
}

/**
 * Según el UART los dos ADC estarían mostrandose correctamente.
 */
static void task_adc(void *args __attribute__((unused))) {
  while (true) {
    // Lectura del canal 0 (temperatura)
    adc_disable_scan_mode(ADC1); // Asegura que no esté en modo escaneo
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // Configura canal 0
    adc_start_conversion_direct(ADC1); // Inicia conversión
    while (!adc_eoc(ADC1)) { 
      // Espera el fin de conversión
    }
   // temperatura = adc_read_regular(ADC1) & 0xFFF; // Lee el valor de 12 bits

    // Lectura del canal 1 (porcentaje de batería)
    adc_disable_scan_mode(ADC1); // Asegura que no esté en modo escaneo
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL1); // Configura canal 1
    adc_start_conversion_direct(ADC1); // Inicia conversión
    while (!adc_eoc(ADC1)) { 
      // Espera el fin de conversión
    }
   // porcentajeBateria = adc_read_regular(ADC1) & 0xFFF; // Lee el valor de 12 bits

    // Verifica la temperatura y activa/desactiva alarma
    if (temperatura < UMBRAL_TEMP_C) { 
      gpio_set(GPIOC, GPIO13); // Activa GPIO (sin alarma)
    } else {
      gpio_clear(GPIOC, GPIO13); // Desactiva GPIO
    }

    // Retraso de 1 segundo antes de la siguiente iteración
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Es importante que el dma este configurado antes de utilizarse !!!
 */
void adc1_2_isr(void) {
    if (adc_eoc(ADC1)) { // Verifica si la conversión ha terminado
        ADC_SR(ADC1) &= ~ADC_SR_EOC; // Limpia la bandera EOC

        // Habilita el canal DMA para transferir el valor del ADC
        dma_enable_channel(DMA1, DMA_CHANNEL1);
    }
}

void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {   // Verifica si la interrupción fue generada por el flag de actualización
    timer_clear_flag(TIM2, TIM_SR_UIF);     // Limpia el flag de interrupción de actualización

    adc_start_conversion_direct(ADC1);       // Inicia la conversión

  }
}

void configure_dma(void){
    // Habilita el reloj para el DMA1 y GPIOC
    rcc_periph_clock_enable(RCC_DMA1);
    // Configura el dma para transferir datos 
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    // Origen de los datos a transferir:
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    // Dirección de memoria destino: variables para almacenar los valores
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_values);

    // Tamaño del dato a leer y escribir: 16 bits (ya que el ADC es de 12 bits)
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    // Cantidad de datos a transferir:
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 2);
    // Incrementa la posición en memoria automáticamente
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    // La dirección del periférico (ADC) se mantiene fija
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);

    // Se establece la prioridad del canal 7 del DMA1 como alta:
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);

    //Se habilita el modo circular para que la transferencia se repita indefinidamente
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

    // Se habilita la interrupción que se ejecutan al finalizar la transferencia
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

    // Habilita la interrupción del canal correspondiente en el NVIC
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    // Habilita el canal de DMA
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    // Relaciona las variables con los valores leídos en el bucle principal o interrupciones
    temperatura = adc_values[0];         // Valor del canal 0
    porcentajeBateria = adc_values[1];  // Valor del canal 1
}

// Interrupción para manejar el fin de la transferencia de DMA
void dma1_channel1_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        // Aquí puedes procesar los valores directamente
        temperatura = adc_values[0];
        porcentajeBateria = adc_values[1];
    }
}


int main(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]); // Configura el reloj a 72 MHz
  
  configure_pins();
  configure_timer(); // Configuración del Timer 2 para sincronizar el muestreo del ADC
  configure_dma();   // Configuración del DMA
  configure_adc();
  configure_usart();

  // ENCENDEMOS el led
  gpio_set(GPIOC, GPIO13);

  xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL,
              tskLOW_PRIORITY, NULL);

  // creamos la tarea ADC
  xTaskCreate(task_adc, "ADC", STACK_SIZE, NULL, tskLOW_PRIORITY_ADC, NULL);

  // iniciamos todas las tareas
  vTaskStartScheduler();

  while (1) {
  }

  return 0;
}


