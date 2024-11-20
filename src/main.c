/** @file main.c
 *
 * @brief Firmware principal para el control de la puerta de un refugio de supervivencia
 *
 */

// Librerias std
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// Librerias Auxiliares
#include "lcd_i2c.h"
#include "config.h"

// Definiciones para buffer
#define BUFFER_SIZE 16

// Enum para el estado de la puerta
enum EstadoPuerta
{
  ABRIENDO,
  CERRANDO,
  DETENIDA
};

// variables globales
volatile uint32_t systick_Count = 0;      // Contador de systick para el control de tiempo
volatile float temperatura = 0;        // Variable para valor de temperatura convertida
volatile uint16_t adc_temperatura=0;  // Variable para almacenar el valor del adc de temperatura
volatile float bateria = 0;            // Variable para almacenar la bateria
volatile uint16_t adc_bateria=0; // Variable para almacenar el valor del adc de bateria
volatile uint16_t Timer_Batery_Count = 0; // contador para leer la bateria
volatile uint16_t Timer_UART_Count = 0;   // contador para mostrar UART
volatile bool flag_UART = 1;              // bandera para enviar datos por UART
volatile bool alarma = false;             // bandera de control para la ALARMA
volatile bool bandera=true;
volatile bool flag_adc_ready=false;

volatile uint32_t contador_puerta = 0; // Varibables para logica de puerta

enum EstadoPuerta estado_puerta = DETENIDA;

bool modo = 1; // 0: manual, 1: automatico

bool sensorMovimiento=false;

char buffer[BUFFER_SIZE];



void exti4_isr(void)
{
  exti_reset_request(EXTI4);  // Limpia la solicitud de interrupción

  if (modo)
  {
    modo = 0;
  }
  else
  {
    modo = 1;
  }
}
void exti3_isr(void)
{
  exti_reset_request(EXTI3);  // Limpia la solicitud de interrupción
  delay_ms(50);
  if(gpio_get(GPIOA, GPIO3)){
    sensorMovimiento=true;
  }
  else{
    sensorMovimiento=false;
  }
}




void sys_tick_handler(void)
{
  systick_Count++;
  if (systick_Count > TOGGLE_COUNT)
  {
    systick_Count = 0;
    print_lcd();
    
  }
  // Logica de puerta
  if (modo == 1)
  {
    if (estado_puerta == ABRIENDO)
    {
      contador_puerta++;
      abrir_puerta();
      if (contador_puerta > TOGGLE_COUNT * 4)
      {
        estado_puerta = DETENIDA;
        parar_puerta();
        contador_puerta = 0;
      }
    }
    else
    {
      if (estado_puerta == CERRANDO)
      {
        contador_puerta++;
        cerrar_puerta();
        if (contador_puerta > TOGGLE_COUNT * 6)
        {
          estado_puerta = DETENIDA;
          parar_puerta();
          contador_puerta = 0;
        }
      }
    }
  }
}

void abrir_puerta()
{
  gpio_clear(GPIOA, GPIO6);
  gpio_set(GPIOA, GPIO7);
}
void cerrar_puerta()
{
  gpio_set(GPIOA, GPIO6);
  gpio_clear(GPIOA, GPIO7);
}
void parar_puerta()
{
  gpio_clear(GPIOA, GPIO6);
  gpio_clear(GPIOA, GPIO7);
}
void activarAlarma()
{
  gpio_set(GPIOB, GPIO8);
  delay_ms(250);
  gpio_clear(GPIOB, GPIO8);
  delay_ms(250);
}


void procesarDatos(uint16_t temp, bool sensorMovimiento)
{
  if ((temp>45 || temp<1)|| sensorMovimiento)
  {
    activarAlarma();
    if(bandera && modo){
      estado_puerta=CERRANDO;
      bandera=false;
    }
  }
  else{
    if(!bandera && modo){
      estado_puerta=ABRIENDO;
      bandera=true;
    }
  }
}

int main(void)
{
  configure_hardware();
  lcd_init();
  configure_dma(&adc_bateria);
  adc_start_conversion_direct(ADC1);
  uint16_t pwm_val=0;
  while (1)
  {
    if (flag_UART)
    {
      enviar_sados();
      flag_UART = 0;
    }
    if(flag_adc_ready){
      flag_adc_ready=false;
      convertirDatos();
      int pwm_val = (adc_bateria * MAX_COUNT) / ADC_MAX_VALUE;
      timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
      procesarDatos(temperatura, sensorMovimiento);
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
  sprintf(buffer, "Temp:%3d Bat:%3d", (int)temperatura,
              (int)bateria);
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print(buffer);

  if (modo == 0) {
        sprintf(buffer, "Modo: %-6s","Manual");
      } else {
        sprintf(buffer, "Modo: %-6s","Automatico");
      }

  // preparamos modo:
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
    if ((Timer_Batery_Count == BATERY_SENSE_TIME))
    {
      Timer_Batery_Count = 0;
      adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1
      dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_bateria);         // Dirección de memoria destino
      adc_power_on(ADC1);                                           // Enciende el ADC
      adc_start_conversion_direct(ADC1);                            // Inicia la conversión
      dma_enable_channel(DMA1, DMA_CHANNEL1);                       // Habilita el canal 1 del DMA1
      Timer_Batery_Count = 0;
    }
    else
    {
      adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL0}); // Secuencia de 1 canal: canal 1
      dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_temperatura);     // Dirección de memoria destino
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
  //pasar de valor de adc a temperatura °c
  flag_adc_ready=true;
  //temperatura= ((float)adc_temperatura / 4095) * 80 - 20;
  //  usart_send_value("Bat:",bateria);  // Envía el valor del ADC por el puerto serial
  //  usart_send_value("Temp:",temperatura);  // Envía el valor del ADC por el puerto serial
}

void convertirDatos(){
   temperatura = ((float)adc_temperatura / 4095) * 80 - 20;
   bateria = ((adc_bateria * 100 / 4095));
  bateria = bateria > 100 ? 100 : bateria < 0 ? 0 : bateria;
  flag_adc_ready=false;
}

void enviar_sados(void)
{
  usart_send_value("Bat:", bateria);      // Envía el valor del ADC por el puerto serial
  usart_send_value("Temp:", temperatura); // Envía el valor del ADC por el puerto serial
}
