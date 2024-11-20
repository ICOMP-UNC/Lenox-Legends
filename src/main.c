/** @file main.c
 *
 * @brief Programa principal del proyecto
 *
 */
// Librerias std
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
// Librerias libopencm3
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

// Librerias Auxiliares
#include "lcd_i2c.h"

#define RELOAD_COUNT 8999 /**< Reload value para el systick */
#define TOGGLE_COUNT 500  /**< Valor a acanzar para que el systick cumpla 500ms */
// Definiciones para el timer
#define TIMER_PRESCALER 7199  /**< Prescaler del Timer */
#define TIMER_PERIOD_ADC 9999 /**< Periodo del Timer para adc */
#define BATERY_SENSE_TIME 3   /**< Tiempo de lectura de la bateria (3 segundos) */
// Definiciones para el USART
#define USART_BAUDRATE 9600 /**< Baudrate del USART */
#define USART_DATABITS 8    /**< Bits de datos del USART */
// Definiciones para la LCD
#define BUFFER_SIZE 20 /**< Tamanio del buffer para la LCD */
// exti
#define FALLING 0 /**< Definicion de FALLING para las exti */
#define RISING 1  /**< Definicion de RISING para las exti */
// Definiciones para el ADC
#define MAX_COUNT 10000    /**< Valor maximo de conteo */
#define ADC_MAX_VALUE 4095 /**< Valor maximo del ADC */

// puerta
#define TIEMPO_MAX_PUERTA TOGGLE_COUNT * 2 /**< Tiempo maximo de la puerta. 1 Segundo */
// TIEMPOS
#define TIEMPO_MAX_UART 10 /**< Tiempo maximo de UART. 10 segundos */

#define BUFFER_SIZE_UART 20    /**< Tamanio del buffer para la UART */
#define TIME_TOGGLE_BUZZER 250 /**< Tiempo de toggle del buzzer. 0.25 seg */

// RANGOS DE TEMPERATURA
#define TEMP_MIN 1  /**< Rango minimo de la temperatura */
#define TEMP_MAX 45 /**< Rango maximo de la temperatura */

/**
 * @brief Enum necesario para tener un control absoluto de la puerta
 *
 */
enum EstadoPuerta
{
    /** @brief Enum para abrir la puerta */
    ABRIENDO, /**< La puerta se esta abriendo */
    /** @brief Enum para cerrar la puerta */
    CERRANDO, /**< La puerta se esta cerrando */
    /** @brief Enum para detener la puerta */
    DETENIDA /**< La puerta se encuentra detenida */
};

// variables globales
volatile uint32_t contador_lcd = 0;           /**< Contador para mostrar por LCD */
volatile float temperatura = 0;               /**< Variable para almacenar la temperatura convertida */
volatile uint16_t adc_temperatura = 0;        /**< Variable para almacenar el valor del adc de la temperatura */
volatile float porcentaje_bateria = 0;        /**< Variable para almacenar el porcentaje de bateria */
volatile uint16_t adc_bateria = 0;            /**< Variable para almacenar el valor del adc de la bateria */
volatile uint16_t contador_timer_bateria = 0; /**< Contador del timer para la lectura de la bateria */
volatile uint16_t contador_timer_uart = 0;    /**< Contador del timer para muestra de uart */
volatile bool flag_UART = 1;                  /**< Bandera de control para UART */
volatile bool alarma = false;                 /**< Bandera de control para la alarma */
// Varibables para logica de puerta
volatile uint32_t contador_puerta = 0;      /**< Contador para logica de puerta en systick */
enum EstadoPuerta estado_puerta = DETENIDA; /**< Variable para almacenar el estado de la puerta */
bool bandera_puerta = true;                 /**< Bandera de control de cierre y apertura de la puerta */

/**
 * @brief  Variable para controlar el modo del sistema
 *        0: Modo normal
 *        1: Modo de alarma
 */
bool modo_sistema = true;

bool sensorMovimiento = false; /**< Variable para controlar el sensor de movimiento */
char buffer[BUFFER_SIZE];      /**< Buffer para la LCD */

// prototipos de funciones

/**
 * @brief Inicializa el clock del sistema a 72MHz
 *
 * @param void
 */
void systemInit();

/**
 * @brief Habilita el clock de los puertos y configura los pines de entrada/salida de la siguiente forma:
 * A00: Canal 0 del ADC para el sondeo de temperatura
 * A01: Canal 1 del ADC para el sondeo de bateria
 * A03: Pin de entrada del sensor de movimiento. EINT3
 * A04: Pin de entrada del boton de cammbio de modo. EINT4
 * A09: TX de UART
 * A10: RX de UART
 * B06: Control 1 de motor
 * B07: Control 2 de motor
 * B09: Salida PWM
 *
 * C13: Led 13
 *
 * @param void
 *
 */
void configure_pins();

/**
 * @brief Configura el systick
 *
 * @param void
 */
void configure_systick(void);

/**
 * @brief Configura el pwm
 *
 * @param void
 */
void configure_pwm();

/**
 * @brief Configura las interrupciones externas.
 *
 * Configura las interrupciones externas para el sensor de movimiento y
 * el boton de cambio de modo
 *
 * @param void
 *
 */
void exti_setup();

/**
 * @brief Configura el timer 2 y su respectiva interrupcion
 *
 * @param void
 *
 */
void configure_timer();
/**
 * @brief Configura y activa las interrupciones por adc.
 * configura los canales ADC_CHANNEL0 y ADC_CHANNEL1
 *
 * @param void
 *
 */
void configure_adc();
/**
 * @brief Configura el dma para poder usar los canales ADC
 * Los datos de temperatura y nivel de bateria seran llevados a memoria
 * a traves del dma.
 *
 * @param void
 *
 */
void configure_dma();
/**
 * @brief Configura el USART
 *
 * @param void
 *
 */
void configure_usart();
/**
 * @brief Envia un mensaje por UART
 *
 * @param label : Etiqueta del mensaje
 * @param value : Valor del mensaje
 */
void usart_send_value(const char* label, uint16_t value);
/**
 * @brief Imprime el estado de la puerta en la LCD
 *
 * @param void
 *
 */
void print_lcd(void);
/**
 * @brief Funcion encargada de abrir la puerta
 *
 * Activa uno de los dos pines de control de la puerta, para que el
 * motor gire en sentido
 *
 * @param void
 */
void abrir_puerta();
/**
 * @brief Funcion encargada de cerrar la puerta
 *
 * Activa uno de los dos pines de control de la puerta, para que el
 * motor gire en otro sentido
 *
 */
void cerrar_puerta();

/**
 * @brief Funcion encargada de parar la puerta
 *
 * Desactiva los pines de control de la puerta
 *
 * @param void
 */
void parar_puerta();
/**
 * @brief Activa el buzz de la alarma.
 *
 * hace un toggle del pin del buzz cada 250ms
 *
 * @param void
 */
void activarAlarma();
/**
 * @brief Funcion encargada de procesar los datos de temperatura y
 * nivel de bateria para tomar desiciones a respecto.
 *
 * @param temp : Temperatura medida por el adc (ya convertida a grados celsius)
 * @param sensorMovimiento : Bandera que indica si el sensor de movimiento esta activo
 */
void procesarDatos(uint16_t temp, bool sensorMovimiento);

/**
 * @brief Enviar los datos de temperatura y nivel de bateria por UART
 *
 * @param void
 */
void enviar_datos_uart(void);

//-------------------------------Implementacion de Funciones----------------------------------

void systemInit()
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_pins()
{
    // Habilitacion de reloj de puertos
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // configuracion pin PWM
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

    // Configuracion Led 13
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_clear(GPIOC, GPIO13);

    // Configuracion entrada ADC
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);

    // configuracion de pines USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX);                                               // Configura PA9 como TX7
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX); // Configura PA10 como RX7

    // configuracion de pines para la puerta
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
    gpio_clear(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);

    // configuracion pin para Boton de Modo
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);

    // configuracion pin para sensor movimiento
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);

    // configuracion de pin para el Buzzer de la alarma
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_clear(GPIOB, GPIO8);
}

void configure_systick(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); // Reloj del sistema de 36 MHz
    systick_set_reload(RELOAD_COUNT);                    // Reload Count para systick
    systick_interrupt_enable();                          // Habilita la interrupción del systick
    systick_counter_enable();                            // Habilita el contador del systick
}
void configure_pwm()
{
    rcc_periph_clock_enable(RCC_TIM4);
    timer_set_mode(TIM4,                 // Timer general 4
                   TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
                   TIM_CR1_CMS_CENTER_1, // Modo centrado
                   TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

    timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz
    timer_set_prescaler(TIM4, 1);

    // Configura el canal PWM para el LED en PB9
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1); // PWM1: activo alto
    timer_enable_oc_output(TIM4, TIM_OC4);          // Habilitar salida OC2

    timer_enable_preload(TIM4);
    timer_generate_event(TIM4, TIM_EGR_UG);
    // Activa el contador del timer
    timer_enable_counter(TIM4);
}
void exti_setup()
{
    rcc_periph_clock_enable(RCC_AFIO);             // Habilita el reloj para AFIO
    nvic_enable_irq(NVIC_EXTI4_IRQ);               // Habilita la interrupción del EXTI4 en el NVIC
    exti_select_source(EXTI4, GPIO4);              // Selecciona el pin AO4 como fuente de interrupción
    exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING); // Configura el trigger de la interrupción
    exti_enable_request(EXTI4);                    // Habilita la solicitud de interrupción

    nvic_enable_irq(NVIC_EXTI3_IRQ);            // Habilita la interrupción del EXTI4 en el NVIC
    exti_select_source(EXTI3, GPIO3);           // Selecciona el pin AO4 como fuente de interrupción
    exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH); // Configura el trigger de la interrupción
    exti_enable_request(EXTI3);                 // Habilita la solicitud de interrupción
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
    adc_set_regular_sequence(
        ADC1, 1,
        (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1  // Configura la secuencia regular de canales

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
    dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_bateria);
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
    rcc_periph_clock_enable(RCC_USART1);                    // Habilita USART1
    usart_set_baudrate(USART1, USART_BAUDRATE);             // Configura el baudrate
    usart_set_databits(USART1, USART_DATABITS);             // Configura la cantidad de bits de datos
    usart_set_stopbits(USART1, USART_STOPBITS_1);           // Configura la cantidad de bits de parada
    usart_set_mode(USART1, USART_MODE_TX);                  // Configura el modo de transmisión
    usart_set_parity(USART1, USART_PARITY_NONE);            // Configura la paridad
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE); // Configura el control de flujo
    usart_enable(USART1);                                   // Habilita USART1
}

//-----------------------------Manejo de interrupciones-------------------------------------
void exti4_isr(void)
{
    exti_reset_request(EXTI4); // Limpia la solicitud de interrupción

    if (modo_sistema)
    {
        modo_sistema = 0;
    }
    else
    {
        modo_sistema = 1;
    }
}
void exti3_isr(void)
{
    exti_reset_request(EXTI3); // Limpia la solicitud de interrupción
    sensorMovimiento = true;
}

void sys_tick_handler(void)
{
    contador_lcd++;
    if (contador_lcd > TOGGLE_COUNT)
    {
        contador_lcd = 0;
        print_lcd();
    }
    // Logica de puerta
    if (modo_sistema)
    {
        if (estado_puerta == ABRIENDO)
        {
            contador_puerta++;
            abrir_puerta();
            if (contador_puerta > TIEMPO_MAX_PUERTA)
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
                if (contador_puerta > TIEMPO_MAX_PUERTA)
                {
                    estado_puerta = DETENIDA;
                    parar_puerta();
                    contador_puerta = 0;
                }
            }
        }
    }
}
void tim2_isr(void)
{
    contador_timer_bateria++;
    contador_timer_uart++;
    if (contador_timer_uart > TIEMPO_MAX_UART)
    {
        flag_UART = 1;
        contador_timer_uart = 0;
    }
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {                                       // Verifica si la interrupción fue generada por el flag de actualización
        timer_clear_flag(TIM2, TIM_SR_UIF); // Limpia el flag de interrupción de actualización
        adc_power_off(ADC1);                // Apaga el ADC para configurarlo
        dma_disable_channel(DMA1, DMA_CHANNEL1); // Deshabilita el canal 1 del DMA1

        // Verifica si el contador es mayor a 5
        if ((contador_timer_bateria == BATERY_SENSE_TIME))
        {
            contador_timer_bateria = 0;
            adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL1}); // Secuencia de 1 canal: canal 1
            dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_bateria);     // Dirección de memoria destino
            adc_power_on(ADC1);                                           // Enciende el ADC
            adc_start_conversion_direct(ADC1);                            // Inicia la conversión
            dma_enable_channel(DMA1, DMA_CHANNEL1);                       // Habilita el canal 1 del DMA1
        }
        else
        {
            adc_set_regular_sequence(ADC1, 1, (uint8_t[]){ADC_CHANNEL0}); // Secuencia de 1 canal: canal 1
            dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_temperatura); // Dirección de memoria destino
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
    // pasar de valor de adc a temperatura °c
    temperatura = ((float)adc_temperatura / ADC_MAX_VALUE) * 80.0f - 20.0f;

    // pasar de valor de adc a bateria %
    porcentaje_bateria = ((adc_bateria * 100 / ADC_MAX_VALUE));
    if (porcentaje_bateria > 100.0f)
    {
        porcentaje_bateria = 100.0f;
    }
    if (porcentaje_bateria < 0.0f)
    {
        porcentaje_bateria = 0.0f;
    }

    //  usart_send_value("Bat:",bateria);  // Envía el valor del ADC por el puerto serial
    //  usart_send_value("Temp:",temperatura);  // Envía el valor del ADC por el puerto serial
}

int main(void)
{
    // CONFIGURACION DE HARDWARE
    systemInit();
    lcd_init();
    configure_pins();
    configure_systick();
    configure_timer();
    configure_adc();
    configure_dma();
    configure_usart();
    exti_setup();
    configure_pwm();
    adc_start_conversion_direct(ADC1);
    // variable auxiliar para pwm
    uint16_t pwm_val = 0;
    while (true)
    {
        if (flag_UART)
        {
            enviar_datos_uart();
            flag_UART = 0;
        }
        pwm_val = (adc_bateria * MAX_COUNT) / ADC_MAX_VALUE;
        timer_set_oc_value(TIM4, TIM_OC4, pwm_val);
        procesarDatos(temperatura, sensorMovimiento);
    }
}

void usart_send_value(const char* label, uint16_t value)
{
    char buffer[BUFFER_SIZE_UART];
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
    sprintf(buffer, "Temp:%3d Bat:%3d", (int)temperatura, (int)porcentaje_bateria);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(buffer);

    if (!modo_sistema)
    {
        sprintf(buffer, "Modo: %-6s", "Manual");
    }
    else
    {
        sprintf(buffer, "Modo: %-6s", "Auto");
    }

    // preparamos modo:
    lcd_set_cursor(1, 0);
    lcd_print(buffer);
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
    delay_ms(TIME_TOGGLE_BUZZER);
    gpio_clear(GPIOB, GPIO8);
    delay_ms(TIME_TOGGLE_BUZZER);
    if (gpio_get(GPIOA, GPIO3))
    {
        sensorMovimiento = false;
    }
}

void procesarDatos(uint16_t temp, bool sensorMovimiento)
{
    if ((temp > TEMP_MAX || temp < TEMP_MIN) || sensorMovimiento)
    {
        activarAlarma();
        if (bandera_puerta && modo_sistema)
        {
            estado_puerta = CERRANDO;
            bandera_puerta = false;
        }
    }
    else
    {
        if (!bandera_puerta && modo_sistema)
        {
            estado_puerta = ABRIENDO;
            bandera_puerta = true;
        }
    }
}

void enviar_datos_uart(void)
{
    usart_send_value("Bat:", porcentaje_bateria); // Envía el valor del ADC por el puerto serial
    usart_send_value("Temp:", temperatura);       // Envía el valor del ADC por el puerto serial
}
