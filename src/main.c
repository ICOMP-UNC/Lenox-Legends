
/**
 * @file main.c
 * @brief Archivo fuente principal para el proyecto Lenox Legends.
 *
 * Este archivo contiene la función principal y las funciones de inicialización y configuración
 * para el proyecto Lenox Legends. Incluye la configuración de periféricos como GPIO, USART, TIMER, ADC, DMA, PWM y EXTI, así como
 * la creación de tareas y semáforos de FreeRTOS.
 *
 * @section Libraries
 * - libopencm3: Proporciona acceso de bajo nivel a los periféricos STM32.
 * - FreeRTOS: Sistema operativo en tiempo real para la gestión de tareas.
 *
 */

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
xSemaphoreHandle cerrar_semaphore = NULL;
xSemaphoreHandle abrir_semaphore = NULL;

// variables Globales
volatile uint16_t temperatura = 0, porcentajeBateria = 0; // Variables para el ADC de temperatura y batería
volatile float temperatura_C = 0, bateria_porcetaje = 0;
bool modo_sistema = 0;                    // Variable para el modo de sistema (0: manual, 1: automático)
uint16_t sensor_movimiento = 0;           // Variable para el sensor de movimiento
volatile uint16_t Timer_Batery_Count = 0; // contador para leer la bateria cada 10 seg
volatile bool Puerta = 1;                 // Variable para el estado de la puerta (0: cerrada, 1: abierta)

char buffer[BUFFER_SIZE]; // Buffer para la impresión de datos

char buffer_temp_bateria[20]; // Buffer para la temperatura y batería
char buffer_modo[24];         // Buffer para el modo de sistema

// ------------------------- Configuración
// ------------------------------------------------
/**
 * @brief Función para configurar los pines del microcontrolador.
 */
void configure_pins()
{
  // Habilita el reloj para los puertos A, B y C
  rcc_periph_clock_enable(RCC_GPIOA); // Habilita GPIOA para USART
  // Configura PA9 como TX y PA10 como RX para USART1
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX); // Configura PA9 como TX

  // configuracion pin para Boton de Modo de sistema
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);
  // configuracion pin para sensor movimiento
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);
  // configuracion de pines para la puerta
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO6);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO7);

  // SE FUERZA LOS PINES DE LOS MOTORES A 0
  gpio_clear(GPIOA, GPIO6);
  gpio_clear(GPIOA, GPIO7);

  rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
  // Configurar B8 como salida para el buzzer
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO8);
  // Configurar B9 para PWM
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO9);

  rcc_periph_clock_enable(RCC_GPIOC);
  // Led de testeo
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);
}

/**
 * @brief Función para configurar el USART.
 */
void configure_usart(void)
{
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
 * @brief Función para configurar el timer 2 y 3.
 */
void configure_timer(void)
{
  // Configuración del Timer 2
  rcc_periph_clock_enable(RCC_TIM2); // Enable TIM2 clock
  timer_disable_counter(TIM2);
  // timer_reset(TIM2); // Reset TIM2 configuration
  timer_set_prescaler(TIM2, 7200 - 1); // Prescaler for 10 kHz timer clock
  timer_set_period(TIM2, 200 - 1);     // Period for 1-second interrupt
  // timer_enable_update_event(TIM2);    // Enable update events
  timer_enable_irq(TIM2, TIM_DIER_UIE); // Enable update interrupt
  nvic_enable_irq(NVIC_TIM2_IRQ);       // Enable TIM2 interrupt in NVIC
  timer_enable_counter(TIM2);           // Start the timer

  // Configuración del Timer 3
  rcc_periph_clock_enable(RCC_TIM3);   // Enable Timer 3 clock
  timer_disable_counter(TIM3);         // Ensure timer is stopped during setup
  timer_set_prescaler(TIM3, 7200 - 1); // Prescaler for 10 kHz clock
  timer_set_period(TIM3,
                   300000 - 1);         // Period for 30 seconds (10,000 Hz / 300,000)
  timer_enable_irq(TIM3, TIM_DIER_UIE); // Enable update interrupt
  nvic_enable_irq(NVIC_TIM3_IRQ);       // Enable NVIC interrupt for Timer 3
  timer_enable_counter(TIM3);           // Start the timer
}

/**
 * @brief Función para configurar el ADC.
 */
void configure_adc(void)
{
  rcc_periph_clock_enable(RCC_ADC1); // Habilita el reloj para el ADC1

  adc_power_off(ADC1); // Apaga el ADC para configurarlo

  // Configura el tiempo de muestreo para cada canal

  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); // Esta función establece el tiempo de muestreo para el canal 0
  adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_1DOT5CYC); // Canal 1
  // Configura la secuencia regular de canales
  adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1

  // Configura el ADC para que dispare la transferencia de datos
  adc_enable_dma(ADC1); // Habilita DMA (si es necesario)

  adc_power_on(ADC1); // Enciende el ADC

  adc_reset_calibration(ADC1); // Reinicia la calibración
  adc_calibrate(ADC1);         // Calibra el ADC
}

/**
 * @brief Función para configurar el DMA.
 */
void configure_dma(void)
{
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
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  // Se habilita la interrupción que se ejecutan al finalizar la transferencia
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

  // Se habilita en el NVIC la interrupción del DMA1-CH7
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
}

/**
 * @brief Función para configurar el PWM.
 */
void configure_pwm(void)
{

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

/**
 * @brief Función para configurar el EXTI.
 */
void configure_exti(void)
{
  rcc_periph_clock_enable(RCC_AFIO);          // Habilita el reloj para AFIO
  nvic_enable_irq(NVIC_EXTI4_IRQ);            // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(EXTI4, GPIO4);           // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(EXTI4, EXTI_TRIGGER_BOTH); // Configura el trigger de la interrupción
  exti_enable_request(EXTI4);                 // Habilita la solicitud de interrupción

  nvic_enable_irq(NVIC_EXTI3_IRQ);            // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(EXTI3, GPIO3);           // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH); // Configura el trigger de la interrupción
  exti_enable_request(EXTI3);                 // Habilita la solicitud de interrupción
}

//------------------- Tareas---------------------------------------------------/*
/**
 *@brief Tarea que se encarga de enviar los datos por el puerto serial.
 */
static void task_uart(void *args __attribute__((unused)))
{
  while (true)
  {
    // Envía las variables etiquetadas por UART
    if (xSemaphoreTake(uart_semaphore, portMAX_DELAY) == pdTRUE)
    {
      usart_send_labeled_value(
          "Bat",
          porcentajeBateria); // Envía el valor del ADC por el puerto serial
      usart_send_labeled_value(
          "Temp",
          temperatura); // Envía el valor del ADC por el puerto serial

      usart_send_labeled_value("Modo", modo_sistema);        // Envía el modo de sistema
      usart_send_labeled_value("Sensor", sensor_movimiento); // Envía el sensor de movimiento
    }
  }
}

/**
 * @brief Tarea que se activa de acuerdo al semaforo: adc_dma_semaphore.
 */
static void task_adc_dma(void *args __attribute__((unused)))
{
  while (true)
  {
    // Si el semáforo adc_dma_semaphore está activo
    if (xSemaphoreTake(adc_dma_semaphore, portMAX_DELAY) == pdTRUE)
    {
      Timer_Batery_Count++;                    // Incrementa el contador para leer la batería
      adc_power_off(ADC1);                     // Apaga el ADC para configurarlo
      dma_disable_channel(DMA1, DMA_CHANNEL1); // Deshabilita el canal 1 del DMA1

      // Verifica si el contador es mayor BATERY_SENSE_TIME
      if ((Timer_Batery_Count == BATERY_SENSE_TIME))
      {
        adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1});   // Secuencia de 1 canal: canal 1
        dma_set_memory_address(DMA1, DMA_CHANNEL1, &porcentajeBateria); // Dirección de memoria destino
        adc_power_on(ADC1);                                             // Enciende el ADC
        adc_start_conversion_direct(ADC1);                              // Inicia la conversión
        dma_enable_channel(DMA1, DMA_CHANNEL1);                         // Habilita el canal 1 del DMA1
        Timer_Batery_Count = 0;
      }
      else
      {
        adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL0}); // Secuencia de 1 canal: canal 1
        dma_set_memory_address(DMA1, DMA_CHANNEL1, &temperatura);     // Dirección de memoria destino
        adc_power_on(ADC1);                                           // Enciende el ADC
        adc_start_conversion_direct(ADC1);
        dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
      }
    }
  }
}

/**
 * @brief Tarea que se activa de acuerdo al semaforo: i2c_semaphore.
 */
static void task_i2c(void *args __attribute__((unused)))
{
  while (true)
  {
    if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE)
    {
      sprintf(buffer_temp_bateria, "Temp:%3d Bat:%3d", (int)temperatura_C,
              (int)bateria_porcetaje);
      if (modo_sistema == 0)
      {
        sprintf(buffer_modo, "Modo: %-6s", "Auto"); //?
      }
      else
      {
        sprintf(buffer_modo, "Modo: %-6s", "Manual");
      }
      lcd_set_cursor(0, 0);
      lcd_print(buffer_temp_bateria);
      lcd_set_cursor(1, 0);
      lcd_print(buffer_modo);
    }
  }
}

/**
 * @brief Tarea que sirve para controlar y manejar el sistema.
 */
static void task_control(void *args __attribute__((unused)))
{
  while (true)
  {
    if (xSemaphoreTake(control_semaphore, portMAX_DELAY) == pdTRUE)
    {
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

      xSemaphoreGive(i2c_semaphore);    // Activa el semáforo de I2C
      xSemaphoreGive(alarma_semaphore); // Activa el semáforo de alarma
    }
  }
}

/**
 * @brief Tarea que sirve para activar la alarma y el buzzer, además de controlar la puerta.
 */
static void task_alarma(void *args __attribute__((unused)))
{
  while (true)
  {
    if (xSemaphoreTake(alarma_semaphore, portMAX_DELAY) == pdTRUE)
    {
      // Si la temperatura es mayor a 50°C o menor a -10°C, o si el sensor de movimiento está activo
      while (temperatura_C > MAX_TEMP || temperatura_C < MIN_TEMP ||
             sensor_movimiento == TRUE)
      {
        // Si la puerta está abierta y el modo de sistema es manual entonces cierra la puerta
        if (modo_sistema == 0 && Puerta == 1)
        {
          xSemaphoreGive(cerrar_semaphore);
          Puerta = 0;
        }

        gpio_set(GPIOB, GPIO8);         // Enciende el buzzer en B8
        vTaskDelay(pdMS_TO_TICKS(100)); // Sonido durante 500 ms (ajustable)
        gpio_clear(GPIOB, GPIO8);       // Apaga el buzzer
        vTaskDelay(pdMS_TO_TICKS(100)); // Sonido durante 500 ms (ajustable)

        // Si el sensor de movimiento ya no está activo, entonces desactiva la alarma
        if (gpio_get(GPIOA, GPIO3))
        {
          sensor_movimiento = FALSE;
        }
      }
      // Abre la puerta si la temperatura es menor a 50°C y mayor a -10°C
      if (modo_sistema == 0 && Puerta == 0)
      {
        xSemaphoreGive(abrir_semaphore);
        Puerta = 1;
      }
    }
  }
}

/**
 * @brief Tarea que sirve para controlar el PWM.
 */
static void task_pwm(void *args __attribute__((unused)))
{
  uint16_t pwm_val = 0;

  while (true)
  {
    // Escala el valor del ADC al rango del PWM
    pwm_val = (temperatura * MAX_COUNT) / ADC_MAX_VALUE;

    // Ajusta el ciclo de trabajo del PWM en función del valor del ADC
    timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
  }
}

/**
 * @brief Tarea que sirve para abrir la puerta.
 */
static void task_abrir(void *args __attribute__((unused)))
{
  while (true)
  {
    if (xSemaphoreTake(abrir_semaphore, portMAX_DELAY) == pdTRUE)
    {
      gpio_clear(GPIOA, GPIO6);
      gpio_set(GPIOA, GPIO7);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_clear(GPIOA, GPIO6);
      gpio_clear(GPIOA, GPIO7);
    }
  }
}

/**
 * @brief Tarea que sirve para cerrar la puerta.
 */
static void task_cerrar(void *args __attribute__((unused)))
{
  while (true)
  {
    if (xSemaphoreTake(cerrar_semaphore, portMAX_DELAY) == pdTRUE)
    {
      gpio_clear(GPIOA, GPIO7);
      gpio_set(GPIOA, GPIO6);
      vTaskDelay(pdMS_TO_TICKS(100));
      gpio_clear(GPIOA, GPIO6);
      gpio_clear(GPIOA, GPIO7);
    }
  }
}

// -------------------------- Otras funciones ----------------------------------
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  // Handle stack overflow
  (void)xTask;      // Avoid unused parameter warning
  (void)pcTaskName; // Avoid unused parameter warning

  // Infinite loop for debugging purposes
  while (1)
  {
  }
}

/**
 * @brief Función que envía un valor etiquetado por USART.
 * @param label Etiqueta para el valor.
 * @param value Valor a enviar.
 */
void usart_send_labeled_value(const char *label, uint16_t value)
{
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label,
                     value); // Formato: "Etiqueta:Valor"
  for (int i = 0; i < len; i++)
  {
    usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
  }
}

/**
 * @brief Función que inicializa los semáforos.
 */
void inicializacion_semanforos(void)
{
  i2c_semaphore = xSemaphoreCreateBinary();
  adc_dma_semaphore = xSemaphoreCreateBinary();
  control_semaphore = xSemaphoreCreateBinary();
  alarma_semaphore = xSemaphoreCreateBinary();
  uart_semaphore = xSemaphoreCreateBinary();
  cerrar_semaphore = xSemaphoreCreateBinary();
  abrir_semaphore = xSemaphoreCreateBinary();

  if (i2c_semaphore == NULL || adc_dma_semaphore == NULL ||
      control_semaphore == NULL || alarma_semaphore == NULL ||
      uart_semaphore == NULL || cerrar_semaphore == NULL ||
      abrir_semaphore == NULL)
  {
    while (1)
      ; // Handle semaphore creation failure
  }
}

/**
 * @brief Función que crea las tareas.
 */
void tareas_por_hacer(void)
{
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
  xTaskCreate(task_pwm, "PWM", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_abrir, "abrir", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(task_cerrar, "cerrar", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
}

// -------------------------- Interrupciones -------------------------------
/**
 * @brief Manejo de la interrupción para el timer 2.
 */
void tim2_isr(void)
{
  if (timer_get_flag(TIM2, TIM_SR_UIF))
  {                                     // Check for update interrupt
    timer_clear_flag(TIM2, TIM_SR_UIF); // Clear the interrupt flag
    xSemaphoreGiveFromISR(adc_dma_semaphore,
                          NULL); // Give the semaphore from ISR
  }
}

/**
 * @brief Manejo de la interrupción para el timer 3.
 */
void tim3_isr(void)
{
  if (timer_get_flag(TIM3, TIM_SR_UIF))
  {                                     // Check update interrupt flag
    timer_clear_flag(TIM3, TIM_SR_UIF); // Clear interrupt flag
    xSemaphoreGiveFromISR(uart_semaphore,
                          NULL); // Give the semaphore from ISR
  }
}

/**
 * @brief Manejo de la interrupción para el ADC.
 */
void adc1_2_isr(void)
{
  if (adc_eoc(ADC1))
  {
    // Limpia el flag de fin de conversión (EOC)
    adc_clear_flag(ADC1, ADC_SR_EOC);
  }
}

/**
 * @brief Manejo de la interrupción para el DMA.
 */
void dma1_channel1_isr(void)
{
  //     // Limpia el flag de transferencia completa
  dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CTCIF1);
  xSemaphoreGiveFromISR(control_semaphore, NULL); // Give the semaphore from ISR
}

/**
 * @brief Manejo de interrupción externa para el botón de modo de sistema.
 */
void exti4_isr(void)
{
  exti_reset_request(EXTI4); // Limpia la solicitud de interrupción

  modo_sistema = gpio_get(GPIOA, GPIO4);
}

/**
 * @brief Manejo de interrupción externa para el sensor de movimiento.
 */
void exti3_isr(void)
{
  exti_reset_request(EXTI3); // Limpia la solicitud de interrupción
  sensor_movimiento = TRUE;
}

// -------------------------------- Funcion principal ---------------------
/**
 * @brief Función principal, inicializa y configura los periféricos y crea las tareas.
 */
int main(void)
{

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
  while (1)
  {
  }

  return 0;
}
