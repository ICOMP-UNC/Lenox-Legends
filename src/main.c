#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

#define MAX_COUNT 10000 // Máximo valor a contar del PWM.

volatile uint32_t millis;

/**
 * @brief Delay bloqueante utilizando el SysTick
 * 
 * @param ms duración de la demora
 */
void delay_ms(uint32_t ms)
{
    uint32_t lm = millis;
    while ((millis - lm) < ms);
}

int main()
{
    // CPU = 72Mhz; APB1 = 36Mhz; APB2 = 72Mhz
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Configuración del Systick para interrupción cada 1ms */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(8999);
    systick_interrupt_enable();
    systick_counter_enable();

    /* Configuración del GPIOB y los pines */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(
        GPIOB,                          // Puerto correspondiente
        GPIO_MODE_OUTPUT_2_MHZ,         // Máxima velocidad de switcheo
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, // Función alternativa
        GPIO7);                         // PB7 para el canal rojo

    /* Configuración del TIM4 para PWM centrado */
    rcc_periph_clock_enable(RCC_TIM4);
    timer_set_mode(
        TIM4,                 // Timer general 4
        TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
        TIM_CR1_CMS_CENTER_1, // Modo centrado
        TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

    timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz

    // Configura el canal PWM para el LED rojo en PB7
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM2); // PWM2: activo bajo
    timer_enable_oc_output(TIM4, TIM_OC2);          // Habilitar salida OC2

    // Activa el contador del timer
    timer_enable_counter(TIM4);

    uint16_t pwm_val = 0;
    uint8_t increment = 1;

    while (true)
    {
        timer_set_oc_value(TIM4, TIM_OC2, pwm_val);

        if (pwm_val >= MAX_COUNT - 1)
        {
            increment = -1; // Empieza a disminuir brillo
        }
        else if (pwm_val == 0)
        {
            increment = 1;  // Empieza a aumentar brillo
        }
        pwm_val += increment;

        delay_ms(1); // Pausa de 1ms entre cambios de brillo
    }
}

/**
 * @brief Sub-Rutina de interrupción del SysTick (cada 1ms)
 */
void sys_tick_handler(void)
{
    millis++; // Incrementa el contador de millis
}
