/** @file main.c
 *
 * @brief Programa principal del proyecto
 *
 */
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <stdint.h>
#include <stdio.h>
#include "lcd_i2c.h"

// Definiciones para el systick
#define RELOAD_COUNT 8999
#define TOGGLE_COUNT 500
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

// variables globales
volatile uint32_t systick_Count = 0;
volatile uint16_t temperatura = 0;
volatile uint16_t bateria = 0;
volatile uint16_t Timer_Batery_Count = 0; // contador para leer la bateria
volatile uint16_t Timer_UART_Count = 0;
volatile bool REINICIO = 1;
volatile bool flag_UART = 1; // bandera para enviar datos por UART
//exti
#define FALLING 0
#define RISING 1

static uint16_t exti_direction=FALLING;

//Varibables para logica de puerta
volatile uint32_t contador_puerta=0;

enum EstadoPuerta{
    ABRIENDO,
    CERRANDO,
    DETENIDA
};

enum EstadoPuerta estado_puerta=DETENIDA;

bool modo=1; //0: manual, 1: automatico

char buffer[BUFFER_SIZE];

// prototipos de funciones
/*
 * @brief Inicializa el sistema
 *
 * @return void
 */
void systemInit(void);

/*
 * @brief Configura los pines
 *
 * @return void
 */
void configure_pins(void);

/*
 * @brief Configura el systick
 *
 * @return void
 */
void configure_systick(void);
/*
 * @brief Imprime en el LCD
 *
 * @return void
 */
void print_lcd(void);

/*
 * @brief Envía un valor por USART
 *
 * @param value Valor a enviar
 * @return void
 */
void usart_send_value(const char *label, uint16_t value);

/*
 * @brief Configura el ADC
 *
 * @return void
 */
void configure_adc(void);

/*
 * @brief Configura el timer
 *
 * @return void
 */
void configure_timer(void);

/*
 * @brief Configura el DMA
 *
 * @return void
 */
void configure_dma(void);

/*
 * @brief Configura el USART
 *
 * @return void
 */
void configure_usart(void);

/*
 * @brief Configura el USART
 *
 * @return void
 */
void usart_send_value(const char *label, uint16_t value);
/*
 * @brief Configura el USART
 *
 * @return void
 */
void sys_tick_handler(void);
/*
 * @brief Configura el USART
 *
 * @return void
 */
void tim2_isr(void);
/*
 * @brief Configura el USART
 *
 * @return void
 */
void adc1_2_isr(void);
/*
 * @brief Configura el USART
 *
 * @return void
 */
void dma1_channel1_isr(void);
/*
 * @brief Configura el USART
 *
 * @return void
 */
void tim3_isr(void);

void abrir_puerta();
void cerrar_puerta();
void parar_puerta();

bool a=false;

//implementaciones

void systemInit()
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_pins()
{
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX

    //pines para la puerta
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
    gpio_clear(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);

    //pines para el boton de modo
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);

    // TODO: Agregar configuración de los pines
}

void configure_systick(void)
{
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(RELOAD_COUNT);
  systick_interrupt_enable();
  systick_counter_enable();
}
void exti_setup(){
    rcc_periph_clock_enable(RCC_AFIO);
    nvic_enable_irq(NVIC_EXTI4_IRQ);
    exti_select_source(EXTI4, GPIO4);
    exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI4);
}
void exti4_isr(void){
    exti_reset_request(EXTI4);
    temperatura=0;
    a=!a;
    if(a){;
        estado_puerta=ABRIENDO;
    }
    else{
        estado_puerta=CERRANDO;
    }
}

void configure_timer(void)
{
  // Configuro el timer 2
  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_reset_pulse(RST_TIM2); // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, TIMER_PRESCALER); // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, TIMER_PERIOD_ADC);   // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE); // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);       // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2); // Inicia el contador del Timer 2
}

void configure_adc(void)
{
  rcc_periph_clock_enable(RCC_ADC1); // Habilita el reloj para el ADC1

  adc_power_off(ADC1); // Apaga el ADC para configurarlo

  // Configura el tiempo de muestreo para cada canal

  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); // Canal 0
  adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_1DOT5CYC); // Canal 1
  adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1});   // Secuencia de 1 canal: canal 1  // Configura la secuencia regular de canales

  // Habilita la interrupción de fin de conversión (EOC) del ADC
  adc_enable_eoc_interrupt(ADC1);
  nvic_enable_irq(NVIC_ADC1_2_IRQ); // Habilita la interrupción del ADC en el NVIC

  // Configura el ADC para que dispare la transferencia de datos
  adc_enable_dma(ADC1); // Habilita DMA (si es necesario)

  adc_power_on(ADC1); // Enciende el ADC

  adc_reset_calibration(ADC1); // Reinicia la calibración
  adc_calibrate(ADC1);         // Calibra el ADC
}

void configure_dma(void)
{
  // Habilita el reloj para el DMA1 y GPIOC
  rcc_periph_clock_enable(RCC_DMA1);

  // Configura el dma para transferir datos
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  // Origen de los datos a transferir:
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, &ADC_DR(ADC1));
  // Dirección de datos destino. El ODR (Output Data Register) del GPIOA:
  dma_set_memory_address(DMA1, DMA_CHANNEL1, &bateria);
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
  // Se habilita el modo circular para que la transferencia se repita indefinidamente
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  // Se habilita la interrupción que se ejecutan al finalizar la transferencia para togglear un pin (no es necesario)
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

  // Se habilita en el NVIC la interrupción del DMA1-CH7
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  dma_enable_channel(DMA1, DMA_CHANNEL1); // Habilita el canal 1 del DMA1
}

void configure_usart(void)
{
  rcc_periph_clock_enable(RCC_USART1);        // Habilita USART1
  usart_set_baudrate(USART1, USART_BAUDRATE); // Configura el baudrate
  usart_set_databits(USART1, USART_DATABITS); // Configura la cantidad de bits de datos
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);

void sys_tick_handler(void){
    systick_Count++;
    if(systick_Count > TOGGLE_COUNT){
        systick_Count = 0;
        gpio_toggle(GPIOC, GPIO13);
        print_lcd();
    }
    //Logica de puerta
    if(modo==1){
        if(estado_puerta==ABRIENDO){
            contador_puerta++;
            abrir_puerta();
            if(contador_puerta>TOGGLE_COUNT*6){
                estado_puerta=DETENIDA;
                parar_puerta();
                contador_puerta=0;
            }
        }
        else{
            if(estado_puerta==CERRANDO){
                contador_puerta++;
                cerrar_puerta();
                if(contador_puerta>TOGGLE_COUNT*6){
                    estado_puerta=DETENIDA;
                    parar_puerta();
                    contador_puerta=0;
                }
            }
        }
    }
}


void abrir_puerta(){
    gpio_clear(GPIOA, GPIO6);
    gpio_set(GPIOA, GPIO7);
}
void cerrar_puerta(){
    gpio_set(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);
}
void parar_puerta(){
    gpio_clear(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);
}
void print_lcd(){
    //preparamos las cosas para imprimir
    temperatura++;
    sprintf(buffer,"Temp: %d",temperatura);
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print(buffer);

    //preparamos modo:
    sprintf(buffer,"Mode: %d",modo);
    lcd_set_cursor(1,0);
    lcd_print(buffer);
}

int main(void)
{
  systemInit();
  lcd_init();
  configure_pins();
  configure_systick();
  configure_timer();
  configure_adc();
  configure_dma();
  configure_usart();
  exti_setup();
  while (1)
  {
    if (flag_UART)
    {
      enviar_sados();
      flag_UART = 0;
    }
  }
}

void usart_send_value(const char *label, uint16_t value)
{
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), "%s:%u\n", label,
                     value); // Formato: "Etiqueta:Valor"
  for (int i = 0; i < len; i++)
  {
    usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
  }
}

void print_lcd()
{
  // preparamos las cosas para imprimir
  sprintf(buffer, "Temp: %d", temperatura);
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print(buffer);

  // preparamos modo:
  sprintf(buffer, "Mode: %d", 0);
  lcd_set_cursor(1, 0);
  lcd_print(buffer);
}

void tim2_isr(void)
{
  Timer_Batery_Count = Timer_Batery_Count + 1;
  Timer_UART_Count++;
  if (Timer_UART_Count > 30)
  {
    flag_UART = 1;
    Timer_UART_Count = 0;
  }
  if (timer_get_flag(TIM2, TIM_SR_UIF))
  {                                          // Verifica si la interrupción fue generada por el flag de actualización
    timer_clear_flag(TIM2, TIM_SR_UIF);      // Limpia el flag de interrupción de actualización
    adc_power_off(ADC1);                     // Apaga el ADC para configurarlo
    dma_disable_channel(DMA1, DMA_CHANNEL1); // Deshabilita el canal 1 del DMA1

    // Verifica si el contador es mayor a 5
    if (((REINICIO) || Timer_Batery_Count == BATERY_SENSE_TIME))
    {
      Timer_Batery_Count = 0;
      REINICIO = 0;
      adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1
      dma_set_memory_address(DMA1, DMA_CHANNEL1, &bateria);         // Dirección de memoria destino
      adc_power_on(ADC1);                                           // Enciende el ADC
      adc_start_conversion_direct(ADC1);                            // Inicia la conversión
      dma_enable_channel(DMA1, DMA_CHANNEL1);                       // Habilita el canal 1 del DMA1
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

void adc1_2_isr(void)
{
  if (adc_eoc(ADC1))
  {
    // Limpia el flag de fin de conversión (EOC)
    adc_clear_flag(ADC1, ADC_SR_EOC);
  }
}

// Función de interrupción del DMA1-CH1
void dma1_channel1_isr(void)
{
  //     // Limpia el flag de transferencia completa
  dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CTCIF1);
  //  usart_send_value("Bat:",bateria);  // Envía el valor del ADC por el puerto serial
  //  usart_send_value("Temp:",temperatura);  // Envía el valor del ADC por el puerto serial
}

void enviar_sados(void)
{
  usart_send_value("Bat:", bateria);      // Envía el valor del ADC por el puerto serial
  usart_send_value("Temp:", temperatura); // Envía el valor del ADC por el puerto serial
}
