#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        timer_clear_flag(TIM2, TIM_SR_UIF); // Limpia el flag
        gpio_toggle(GPIOC, GPIO13);        // Alterna el LED
    }
}

void configure_pins(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void configure_timer(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    timer_set_prescaler(TIM2, 7199);
    timer_set_period(TIM2, 9999);

    timer_enable_irq(TIM2, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM2_IRQ);

    timer_enable_counter(TIM2);
}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    configure_pins();
    configure_timer();

    while (1)
    {
        // Bucle principal vacío
    }
}
