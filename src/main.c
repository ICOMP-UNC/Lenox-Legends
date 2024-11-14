/**
 * En este código se controla la señal PWM que sale por el pin B7 mediante la señal analógica
 * leída por el pin A0.
 * El timer 2 interrumpe cada 1 segundo para realizar la conversión por ADC.
 * Los valores leídos por el ADC son cargados a la computadora mediante UART.
 * A su vez la señal PWM se encarga de manejar el brillo de un led.
 * Y podemos observar que para valores bajos de la lectura del ADC la señal PWM tiene un duty cycle grande,
 * mientras que para valores grandes de la lectura del ADC la señal PWM tiene un duty cycle pequeño.
 * Se utiliza otro canal del ADC y al pasar determinado UMBRAL se enciende el led pc13.
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

#define MAX_COUNT 10000     // Máximo valor del PWM
#define ADC_MAX_VALUE 4095  // Valor máximo de 12 bits para el ADC
#define UMBRAL 2000

volatile uint32_t millis;
volatile uint16_t adc_value1 = 0; // Valor leído del ADC
volatile uint16_t adc_value2 = 0; // Valor leído del ADC

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

/**
 * @brief Configuración del ADC en el canal A0.
 */
void configure_adc(void)
{
    rcc_periph_clock_enable(RCC_ADC1);       // Habilita el reloj para el ADC1
    adc_power_off(ADC1);                     // Apaga el ADC para configurarlo
    
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); // Configura el tiempo de muestreo
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_1DOT5CYC);

    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // Selecciona el canal 0 para el ADC (A0)

    adc_power_on(ADC1);                      // Enciende el ADC
    adc_reset_calibration(ADC1);             // Reinicia la calibración
    adc_calibrate(ADC1);                     // Calibra el ADC
}

/**
 * @brief Configuración del Timer 2 para interrumpir cada 1 segundo.
 */
void configure_timer2(void)
{
    rcc_periph_clock_enable(RCC_TIM2); // Habilita el reloj para el Timer 2
    timer_set_prescaler(TIM2, 7200 - 1); // Divide la frecuencia (72 MHz / 7200 = 10 kHz)
    timer_set_period(TIM2, 10000 - 1);   // Cuenta hasta 10000 (1 Hz o 1 segundo)
    timer_enable_irq(TIM2, TIM_DIER_UIE); // Habilita interrupción por actualización
    nvic_enable_irq(NVIC_TIM2_IRQ);       // Activa la interrupción en el NVIC
    timer_enable_counter(TIM2);           // Activa el contador
}

/**
 * @brief Lee el valor actual del ADC en el canal 0.
 * @return Valor del ADC de 12 bits (0 a 4095).
 */
uint16_t read_adc(uint8_t channel) {
    adc_set_regular_sequence(ADC1, 1, channel);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1));
    return adc_read_regular(ADC1);
}

void configure_usart(void) {
    rcc_periph_clock_enable(RCC_USART1);        // Habilita USART1
    rcc_periph_clock_enable(RCC_GPIOA);         // Habilita GPIOA para USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

/**
 * @brief Configura el LED en PC13.
 */
void configure_gpio(void) {
    rcc_periph_clock_enable(RCC_GPIOC);       // Habilita GPIOC para el LED
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); // Configura el LED en PC13
}

void usart_send_value(uint16_t value) {
    char buffer[10];
    int len = snprintf(buffer, sizeof(buffer), "%u\n", value); // Convierte el valor a cadena
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART1, buffer[i]);   // Envía cada carácter
    }
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
        GPIO7);                         // PB7 para el canal de PWM

    /* Configuración del TIM4 para PWM centrado */
    rcc_periph_clock_enable(RCC_TIM4);
    timer_set_mode(
        TIM4,                 // Timer general 4
        TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
        TIM_CR1_CMS_CENTER_1, // Modo centrado
        TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

    timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz

    // Configura el canal PWM para el LED en PB7
    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_PWM2); // PWM2: activo bajo
    timer_enable_oc_output(TIM4, TIM_OC2);          // Habilitar salida OC2

    // Activa el contador del timer
    timer_enable_counter(TIM4);

    // Configura el ADC para el canal A0
    configure_adc();

    // Configura el Timer 2 para interrupción cada 1 segundo
    configure_timer2();

    configure_usart();

    configure_gpio();

    uint16_t pwm_val = 0;

    while (true)
    {
        // Escala el valor del ADC al rango del PWM
        pwm_val = (adc_value1 * MAX_COUNT) / ADC_MAX_VALUE;

        // Ajusta el ciclo de trabajo del PWM en función del valor del ADC
        timer_set_oc_value(TIM4, TIM_OC2, pwm_val);

        usart_send_value(adc_value1);  // Envía el valor del ADC por el puerto serial

        adc_value2 = read_adc(ADC_CHANNEL1);

        if (adc_value2 > UMBRAL) // Verifica si la lectura de A1 supera el UMBRAL
        {
            gpio_clear(GPIOC, GPIO13);  // Enciende el LED (PC13 en bajo)
        }
        else
        {
            gpio_set(GPIOC, GPIO13);    // Apaga el LED (PC13 en alto)
        }

        delay_ms(100); // Pausa entre actualizaciones del PWM
    }

    return 0;
}

/**
 * @brief Sub-Rutina de interrupción del SysTick (cada 1ms)
 */
void sys_tick_handler(void)
{
    millis++; // Incrementa el contador de millis
}

/**
 * @brief Sub-Rutina de interrupción del Timer 2 (cada 1 segundo)
 */
void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF)) // Verifica si ocurrió una actualización
    {
        timer_clear_flag(TIM2, TIM_SR_UIF); // Limpia la bandera de interrupción
        adc_value1 = read_adc(ADC_CHANNEL0);
    }
}