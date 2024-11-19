// -------------------------- Librerías --------------------------------
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "lcd_i2c.h"


// ------------------------- Definiciones ------------------------------



// ------------------------- Variables   ------------------------------
// Create a binary semaphore
xSemaphoreHandle led_semaphore = NULL;

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


// -------------------------------------- Configuración ------------------------------------------------
void configure_pins()
{
    rcc_periph_clock_enable(RCC_GPIOA);  // Habilita GPIOA para USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // TODO: Agregar configuración de los pines
}

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

// Timer configuration
void configure_timer(void) {
    rcc_periph_clock_enable(RCC_TIM2); // Enable TIM2 clock

    timer_disable_counter(TIM2);

    //timer_reset(TIM2); // Reset TIM2 configuration
    timer_set_prescaler(TIM2, 7200 - 1); // Prescaler for 10 kHz timer clock
    timer_set_period(TIM2, 200 - 1); // Period for 1-second interrupt

    //timer_enable_update_event(TIM2);    // Enable update events
    
    timer_enable_irq(TIM2, TIM_DIER_UIE); // Enable update interrupt
    nvic_enable_irq(NVIC_TIM2_IRQ); // Enable TIM2 interrupt in NVIC

    timer_enable_counter(TIM2); // Start the timer
}

//------------------- Tareas --------------------------------------------------------------------
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

// Task to toggle the LED based on the semaphore
static void task_adc_dma(void *args __attribute__((unused))) {
    while (true) {
        // Wait for the semaphore indefinitely
        if (xSemaphoreTake(led_semaphore, portMAX_DELAY) == pdTRUE) {
            // Toggle PC13 state
            gpio_toggle(GPIOC, GPIO13);
        }
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

// ------------------------------------- Otras funciones ---------------------------
// Hook function for stack overflow
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // Handle stack overflow
    (void)xTask;      // Avoid unused parameter warning
    (void)pcTaskName; // Avoid unused parameter warning

    // Infinite loop for debugging purposes
    while (1) {}
}

// // Task to periodically give the semaphore
// static void task_semaphore_giver(void *args __attribute__((unused))) {
//     while (true) {
//         vTaskDelay(pdMS_TO_TICKS(10)); // Wait 1 second
//         xSemaphoreGive(led_semaphore);  // Give the semaphore
//     }
// }

void usart_send_labeled_value(const char *label, uint16_t value)
{
    char buffer[20];
    int len = snprintf(buffer, sizeof(buffer), ">%s:%u\n", label, value); // Formato: "Etiqueta:Valor"
    for (int i = 0; i < len; i++)
    {
        usart_send_blocking(USART1, buffer[i]); // Envía cada carácter
    }
}

// --------------------------------------- Interrupciones ---------------------------
// Timer interrupt handler
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) { // Check for update interrupt
        timer_clear_flag(TIM2, TIM_SR_UIF); // Clear the interrupt flag
        xSemaphoreGiveFromISR(led_semaphore, NULL); // Give the semaphore from ISR
    }
}

// -------------------------------- Funcion principal ---------------------
int main(void) {
    configure_pins();
    configure_usart();

    lcd_init();

    // Create the binary semaphore
    led_semaphore = xSemaphoreCreateBinary();
    if (led_semaphore == NULL) {
        while (1);  // Handle semaphore creation failure
    }

    configure_timer(); // Initialize timer // Cuidado con cambiar el orden!

    // Optionally give semaphore initially
    //xSemaphoreGive(led_semaphore);

    // Create tasks
    xTaskCreate(task_adc_dma, "LED Control", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    xTaskCreate(task_uart, "UART", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(task_i2c, "I2C", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2,
                NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // Infinite loop (should never reach here)
    while (1) {}

    return 0;
}
