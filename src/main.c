#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Create a binary semaphore
xSemaphoreHandle led_semaphore = NULL;

// Task to toggle the LED based on the semaphore
static void task_led_control(void *args __attribute__((unused))) {
    while (true) {
        // Wait for the semaphore indefinitely
        if (xSemaphoreTake(led_semaphore, portMAX_DELAY) == pdTRUE) {
            // Toggle PC13 state
            gpio_toggle(GPIOC, GPIO13);
        }
    }
}

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

// Timer interrupt handler
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) { // Check for update interrupt
        timer_clear_flag(TIM2, TIM_SR_UIF); // Clear the interrupt flag
        xSemaphoreGiveFromISR(led_semaphore, NULL); // Give the semaphore from ISR
    }
}

// Timer configuration
void timer_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2); // Enable TIM2 clock

    timer_disable_counter(TIM2);

    //timer_reset(TIM2); // Reset TIM2 configuration
    timer_set_prescaler(TIM2, 7200 - 1); // Prescaler for 10 kHz timer clock
    timer_set_period(TIM2, 10000 - 1); // Period for 1-second interrupt

    //timer_enable_update_event(TIM2);    // Enable update events
    
    timer_enable_irq(TIM2, TIM_DIER_UIE); // Enable update interrupt
    nvic_enable_irq(NVIC_TIM2_IRQ); // Enable TIM2 interrupt in NVIC

    timer_enable_counter(TIM2); // Start the timer
}

int main(void) {
    // Enable GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set PC13 as output
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // Create the binary semaphore
    led_semaphore = xSemaphoreCreateBinary();
    if (led_semaphore == NULL) {
        while (1);  // Handle semaphore creation failure
    }

    // Initialize timer
    timer_setup();

    // Optionally give semaphore initially
    //xSemaphoreGive(led_semaphore);

    // Create tasks
    xTaskCreate(task_led_control, "LED Control", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(task_semaphore_giver, "Semaphore Giver", configMINIMAL_STACK_SIZE,
    //             NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // Infinite loop (should never reach here)
    while (1) {}

    return 0;
}
