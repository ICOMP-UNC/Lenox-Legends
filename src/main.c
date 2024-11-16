#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdio.h>

#include "lcd_i2c.h"

// Definiciones de pins
// TO DO: Agregar definiciones de los pines

// variables Globales
volatile uint16_t temperatura = 0;
volatile uint16_t porcentajeBateria = 0;
uint8_t modo_sistema = 0;
uint16_t sensor_movimiento = 0;

// SEMAFOROS
xSemaphoreHandle temp_semaforo = 0;
xSemaphoreHandle bateria_semaforo = 0;
xSemaphoreHandle movimiento_semaforo = 0;

char buffer_temp_bateria[32];
char buffer_modo[32];

#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY + 2)

// rutina para controlar el stack overflow
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)),
                                   char *pcTaskName __attribute__((unused)))
{
    while (1)
    {
    }
}

void inicializar_semaforos()
{
    temp_semaforo = xSemaphoreCreateMutex();
    bateria_semaforo = xSemaphoreCreateMutex();
    movimiento_semaforo = xSemaphoreCreateMutex();
}

void configure_pins()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // TODO: Agregar configuración de los pines
}

void configure_usart(void)
{
    rcc_periph_clock_enable(RCC_USART1); // Habilita USART1
    rcc_periph_clock_enable(RCC_GPIOA);  // Habilita GPIOA para USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

void usart_send_labeled_value(const char *label, uint16_t value)
{
    char buffer[20];
    int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label, value); // Formato: "Etiqueta:Valor"
    for (int i = 0; i < len; i++)
    {
        usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
    }
}

// asi se crean las tareas
static void task1(void *args __attribute__((unused)))
{
    while (true)
    {
        gpio_toggle(GPIOC, GPIO13);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_i2c(void *args __attribute__((unused)))
{
    while (true)
    {
        temperatura++;
        porcentajeBateria++;
        modo_sistema = 0;
        sprintf(buffer_temp_bateria, "Temperatura: %d", temperatura);
        if (modo_sistema == 0)
        {
            sprintf(buffer_modo, "Modo: Auto", modo_sistema);
        }
        else
        {
            sprintf(buffer_modo, "Modo: Manual", modo_sistema);
        }
        lcd_set_cursor(0, 0);
        lcd_print(buffer_temp_bateria);
        lcd_set_cursor(1, 0);
        lcd_print(buffer_modo);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void task_uart(void *args __attribute__((unused)))
{
    while (true)
    {
        // Envía las variables etiquetadas por UART
        usart_send_labeled_value("A1", temperatura); // Envía adc_value1 con etiqueta "A1"
        usart_send_labeled_value("A2", porcentajeBateria); // Envía adc_value2 con etiqueta "A2"
        usart_send_labeled_value("V3", sensor_movimiento); // Envía variable3_uart con etiqueta "V3"
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    configure_pins();
    configure_usart();
    lcd_init();

    // ENCENDEMOS el led
    gpio_set(GPIOC, GPIO13);

    // creamos las tareas
    xTaskCreate(task1, "LedSwitching", configMINIMAL_STACK_SIZE, NULL,
                tskLOW_PRIORITY, NULL);
    xTaskCreate(task_i2c, "I2C", configMINIMAL_STACK_SIZE, NULL, tskLOW_PRIORITY,
                NULL);

    xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL, tskLOW_PRIORITY,
                NULL);

    // iniciamos todas las tareas
    vTaskStartScheduler();

    while (1)
    {
    }
}
