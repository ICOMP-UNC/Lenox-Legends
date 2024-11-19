/**
 * En este código la idea es realizar la lectura de datos por medio de dos
 * canales del ADC, el canal 0 y 1. Por el canal 0 sensamos la temperatura
 * utilizando el lm35. Por el canal 1 sensamos el estado de la batería por
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
 * el estado de la batería ya estaría configurado. Y cuando la temperatura supera
 * cierto umbral se enciende el led pc13.
 *
 * Se corrigen las unidades para mostrar en °C los grados sensados y en
 * porcentaje la batería 0-100%. El rango de temperatura será de -20°C a 60°C.
 *
 * El UART funciona correctamente y podemos ver una pequeña discrepancia con
 * respecto al LCD de +-1%.
 *
 * En este código se agrego un buzzer que hace un sonido cuando la temperatura
 * es mayor a la umbral. Se conecto al pin B8.
 *
 * También se agregó una corrección para poder gráficar los valores leídos.
 * 
 * Se coneceta el sensor IR-08H que permite detectar movimiento, y envía una señal digital al 
 * pin B11. Esta señal es procesada y si se detecta movimiento se enciende el led amarillo conectado
 * al pin B13.
 * 
 * Por lo tanto, ya queda conectado el sensor IR 08H y se envía correctamente mediante UART.
 * 
 * Se agrega semaforo que prenda el led amarillo (B13) en una task aparte.
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include <task.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdio.h>
#include <semphr.h>
#include <string.h> // Incluir para usar strlen
#include "lcd_i2c.h"

#define STACK_SIZE configMINIMAL_STACK_SIZE
#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY + 2)
#define tskLOW_PRIORITY_ADC ((UBaseType_t)tskIDLE_PRIORITY + 3)
#define MAX_COUNT 10000    // Máximo valor del PWM
#define ADC_MAX_VALUE 4095 // Valor máximo de 12 bits para el ADC
#define UMBRAL_TEMP_C 20   // Umbral de temperatura en °C

// Definiciones de pines
#define PIN_SENSOR_MOVIMIENTO GPIOB, GPIO11  // B11 para la señal del sensor IR-08H
#define PIN_LED_AMARILLO GPIOB, GPIO13       // B13 para el LED amarillo

// variables Globales
volatile float temperatura_C = 0, bateria_porcetaje = 0;
volatile uint16_t temperatura = 0, porcentajeBateria = 0; // Valores RAW
uint8_t modo_sistema = 0;
uint16_t sensor_movimiento = 0;
static volatile uint16_t adc_values[2]; // Array para los valores del ADC

//Definicion de funciones
void configure_pins(void);
void configure_usart(void);
void configure_timer(void);
void configure_adc(void);
void alarma(void);
void inicializar_semaforos(void);
void usart_send_labeled_value(const char *label, uint16_t value);
void configure_dma(void);
void configure_pwm(void);
void task_adc(void *args);
static void task_uart(void *args);
void task_pwm(void *args);
void task_sensor_movimiento(void *args);
static void task_i2c(void *args);


// SEMAFOROS
xSemaphoreHandle temp_semaforo = 0;
xSemaphoreHandle bateria_semaforo = 0;
xSemaphoreHandle movimiento_semaforo = 0;
xSemaphoreHandle led_semaphore = NULL;

// Hook function for stack overflow
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // Handle stack overflow
    (void)xTask;      // Avoid unused parameter warning
    (void)pcTaskName; // Avoid unused parameter warning

    // Infinite loop for debugging purposes
    while (1) {}
}

void inicializar_semaforos() {
  temp_semaforo = xSemaphoreCreateMutex();
  bateria_semaforo = xSemaphoreCreateMutex();
  movimiento_semaforo = xSemaphoreCreateMutex();
}

void configure_pins() {
  rcc_periph_clock_enable(RCC_GPIOA); // Habilitar el reloj para el puerto A
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1); 
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX

  rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8); // Configurar B8 como salida
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11); // Configurar B11 como entrada flotante para el sensor de movimiento
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); // B13 como salida para el LED amarillo
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); // PB9 para el canal de PWM

  rcc_periph_clock_enable(RCC_GPIOC); // Habilitar el reloj para el puerto C
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //PC13 led de la placa
}

/**
 * En esta función se realiza la configuración del módulo UART.
 * Se lo configura para trabajar a 9600 baudios.
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
 * Permite visualizar las valores por pantalla de la transmisión 
 * por comunicación UART.
 * 
 * También se le da el formato adecuado a las variables para poder graficarse
 * utilizando la extensión Teleplot.
 */
void usart_send_labeled_value(const char *label, uint16_t value) {
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label,
                     value); // Formato: "Etiqueta:Valor"
  for (int i = 0; i < len; i++) {
    usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
  }
}

/**
 * Genera una alarma.
 */
void alarma(void) {
  gpio_set(GPIOB, GPIO8);         // Enciende el buzzer en B8
  vTaskDelay(pdMS_TO_TICKS(500)); // Sonido durante 500 ms (ajustable)
  gpio_clear(GPIOB, GPIO8);       // Apaga el buzzer

  /**
   * Otra musiquita
   */
  //  for (int i = 0; i < 5; i++) {  // Emitir 5 ciclos de sonido
  //   gpio_set(GPIOB, GPIO8);  // Activar el buzzer (pin B8 en alto)
  //   vTaskDelay(pdMS_TO_TICKS(200));  // Esperar 200 ms
  //   gpio_clear(GPIOB, GPIO8);  // Desactivar el buzzer (pin B8 en bajo)
  //   vTaskDelay(pdMS_TO_TICKS(200));  // Esperar 200 ms
  // }
}

void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2);       // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, 7199);         // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, 9999);            // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE);    // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);          // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2);              // Inicia el contador del Timer 2
}

/**
 * Configuración del ADC utilizando dos canales.
 * El canal 0 y 1.
 */
void configure_adc(void) {
  // Habilita el reloj para el ADC1
  rcc_periph_clock_enable(RCC_ADC1);
  rcc_periph_clock_enable(RCC_GPIOA);

  // Apaga el ADC para configurarlo
  adc_power_off(ADC1);

  // Configuro el tiempo de muestreo
  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_55DOT5CYC);
  adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_55DOT5CYC);
  // Selecciono el canal 0
   adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // ?
   //habilito dma
    adc_enable_dma(ADC1);
    //habilito interrupcion
    adc_enable_eoc_interrupt(ADC1);
    nvic_enable_irq(NVIC_ADC1_2_IRQ);
  // Prendo el ADC
  adc_power_on(ADC1);
  // Calibro el ADC
  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
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

// Task to toggle the LED based on the semaphore
static void task_led_control(void *args __attribute__((unused))) {
    while (true) {
        // Wait for the semaphore indefinitely
        if (xSemaphoreTake(led_semaphore, portMAX_DELAY) == pdTRUE) {
            // Toggle PC13 state
            gpio_toggle(GPIOB, GPIO13);
        }
    }
}
/**
 * 
 */
void task_sensor_movimiento(void *args __attribute__((unused))) {
  while (true) {
    // Leer el valor del sensor de movimiento
    sensor_movimiento = gpio_get(GPIOB, GPIO11);  // Leer B11

    // Si el sensor detecta movimiento (asumimos que 1 es detección de movimiento)
    // if (sensor_movimiento) {
    //   gpio_clear(GPIOB, GPIO13);  // Encender LED amarillo
    // } else {
    //   gpio_set(GPIOB, GPIO13);  // Apagar LED amarillo
    // }

    if (!sensor_movimiento) {
      xSemaphoreGive(led_semaphore);  // Give the semaphore
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // Esperar 100 ms antes de volver a leer
  }
}

/**
 * Tarea que permite mostrar en la pantalla LCD 16x2 los valores indicados.
 * En este caso se muestra la temperatura, el nivel de batería, y el modo en que
 * se encuentra el sistema.
 */
static void task_i2c(void *args __attribute__((unused))) {
  while (true) {
    char buffer_temp_bateria[20];
    char buffer_modo[20];
   // temperatura_C=temperatura;
   // bateria_porcetaje=porcentajeBateria;
    // Usar sprintf para formatear los valores flotantes
    sprintf(buffer_temp_bateria, "Temp:%d Bat:%d", (int)temperatura_C,
            (int)bateria_porcetaje); // Convertir la temperatura a cadena

    sprintf(buffer_modo, "Modo: %s",
            (modo_sistema == 0) ? "Auto"
                                : "Manual"); // Convertir el modo a cadena

    lcd_set_cursor(0, 0);
    lcd_print(buffer_temp_bateria);
    lcd_set_cursor(1, 0);
    lcd_print(buffer_modo);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Tarea que lleva a cabo la comunicación UART con el computador.
 * Permita transmitir las variables indicadas.
 */
void task_uart(void *args __attribute__((unused))) {
  while (true) {
    // Enviar la temperatura con un decimal
    usart_send_labeled_value("A1", temperatura);

    // Enviar el porcentaje de batería con un decimal
    usart_send_labeled_value("A2", porcentajeBateria);

    // Enviar el valor del sensor de movimiento (por ejemplo, como entero)
    usart_send_labeled_value("V3", sensor_movimiento);

    vTaskDelay(
        pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de enviar nuevamente
  }
}

/**
 * Según el UART los dos ADC estarían mostrandose correctamente.
 */
void task_adc(void *args __attribute__((unused))) {
  while (true) {
    adc_disable_scan_mode(ADC1); // Deshabilitar modo escaneo para una conversión única
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1))
      ;
    // Convertir el valor del ADC a la temperatura en el rango de -20°C a 60°C
    temperatura_C = ((float)temperatura / 4095.0f) * 80.0f - 20.0f;

    if (temperatura_C > 60.0f)
      temperatura_C = 60.0f;
    if (temperatura_C < -20.0f)
      temperatura_C = 0.0f;

    //Lectura del canal 1
    adc_disable_scan_mode(ADC1); // Asegurar que no queden configuraciones residuales
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL1);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1))
      ;
    // Ajuste del porcentaje de batería, asegurando que no supere el 100%
    bateria_porcetaje = (float)porcentajeBateria * (100.0f / 4095.0f);

    // Limitar el valor del porcentaje de batería a un máximo de 100%
    if (bateria_porcetaje > 100.0f)
      bateria_porcetaje = 100.0f;
    if (bateria_porcetaje < 0.0f)
      bateria_porcetaje = 0.0f;

    if (temperatura_C < UMBRAL_TEMP_C) {
      gpio_set(GPIOC, GPIO13);
    } else {
      gpio_clear(GPIOC, GPIO13);
      alarma();
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Tarea PWM
void task_pwm(void *args __attribute__((unused))) {

  uint16_t pwm_val = 0;

  while (true) {
    // Escala el valor del ADC al rango del PWM
    pwm_val = (temperatura * MAX_COUNT) / ADC_MAX_VALUE;

    // Ajusta el ciclo de trabajo del PWM en función del valor del ADC
    timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
  }
}

/**
 * Función principal.
 */
int main(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  configure_pins();
  configure_usart();
  configure_adc();
  configure_pwm();
  lcd_init();
    configure_dma();
    configure_timer();

  // Create the binary semaphore
    led_semaphore = xSemaphoreCreateBinary();
    if (led_semaphore == NULL) {
        while (1);  // Handle semaphore creation failure
    }


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

  // creamos la tarea ADC
  xTaskCreate(task_sensor_movimiento, "Sensor", configMINIMAL_STACK_SIZE, NULL, tskLOW_PRIORITY, NULL);

  // Create tasks
  xTaskCreate(task_led_control, "LED Control", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2, NULL);

  //xTaskCreate(task_semaphore_giver, "Semaphore Giver", configMINIMAL_STACK_SIZE,
  //              NULL, tskIDLE_PRIORITY + 1, NULL);


  // iniciamos todas las tareas
  vTaskStartScheduler();

  while (1) {
  }

  return 0;
}

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

// Interrupción para manejar el fin de la transferencia de DMA
void dma1_channel1_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        // Aquí puedes procesar los valores directamente
        temperatura = adc_values[0];
        porcentajeBateria = adc_values[1];
    }
}