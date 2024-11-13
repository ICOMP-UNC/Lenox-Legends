/**
 * En este programa se logra leer el valor analógico dado por un potenciometro conectado al pin A0, 
 * y es transmitido por UART a la computadora.
 * Si el valor supera un umbral se enciende el led pc13.
 * Se debe tener en cuenta que para poder hacer lectura del puerto y poder ver los valores del adc, es necesario:
 * 1. Agregar las siguientes líneas en platformio.ini:
 * monitor_speed = 9600
 * monitor_filters = direct
 * 2. Abrir la terminal y ejecutar el siguiente comando: lsof /dev/ttyUSB0
 * 3. Terminar los procesos con: kill -9 xxxxxxx
 * 4. Abrir la lectura del puerto nuevamente con: kill -9 23456
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

void systemInit(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();    // Configuración del reloj a 72 MHz
    rcc_periph_clock_enable(RCC_GPIOC);         // Habilita GPIOC para el LED
    rcc_periph_clock_enable(RCC_ADC1);          // Habilita el ADC1
    rcc_periph_clock_enable(RCC_USART1);        // Habilita USART1
    rcc_periph_clock_enable(RCC_GPIOA);         // Habilita GPIOA para USART
}

void configure_gpio(void) {
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); // Configura el LED en PC13
}

void configure_adc(void) {
    adc_power_off(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_239DOT5CYC); // Tiempo de muestreo
    adc_power_on(ADC1);

    // Calibración del ADC
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
}

void configure_usart(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

uint16_t read_adc(void) {
    adc_start_conversion_direct(ADC1);           // Inicia la conversión
    while (!adc_eoc(ADC1));                      // Espera a que la conversión termine
    return adc_read_regular(ADC1);               // Lee el valor convertido
}

void usart_send_value(uint16_t value) {
    char buffer[10];
    int len = snprintf(buffer, sizeof(buffer), "%u\n", value); // Convierte el valor a cadena
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART1, buffer[i]);   // Envía cada carácter
    }
}

int main(void) {
    systemInit();
    configure_gpio();
    configure_adc();
    configure_usart();

    while (1) {
        uint16_t adc_value = read_adc();
        usart_send_value(adc_value);  // Envía el valor del ADC por el puerto serial

        if (adc_value > 2048) {
            gpio_set(GPIOC, GPIO13);   // Enciende LED si es mayor al umbral
        } else {
            gpio_clear(GPIOC, GPIO13); // Apaga LED si es menor al umbral
        }
    }

    return 0;
}
