#include "config.h"

void systemInit()
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_pins()
{
  //Habilitacion de reloj de puertos
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  //configuracion pin PWM
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

  //Configuracion Led 13
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_clear(GPIOC, GPIO13);
  
  //Configuracion entrada ADC
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);

  //configuracion de pines USART
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // Configura PA9 como TX7
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);                 // Configura PA10 como RX7

  //configuracion de pines para la puerta
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
  gpio_clear(GPIOA, GPIO6);
  gpio_clear(GPIOA, GPIO7);

  //configuracion pin para Boton de Modo
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);

  //configuracion pin para sensor movimiento
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);

  //configuracion de pin para el Buzzer de la alarma
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
  gpio_clear(GPIOB, GPIO8);
}

void configure_systick(void)
{
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);  // Reloj del sistema de 36 MHz
  systick_set_reload(RELOAD_COUNT); // Reload Count para systick
  systick_interrupt_enable(); // Habilita la interrupción del systick
  systick_counter_enable(); // Habilita el contador del systick
}
void configure_pwm(){
  rcc_periph_clock_enable(RCC_TIM4);
  timer_set_mode(TIM4,                 // Timer general 4
                 TIM_CR1_CKD_CK_INT,   // Clock interno como fuente
                 TIM_CR1_CMS_CENTER_1, // Modo centrado
                 TIM_CR1_DIR_UP);      // Dirección del conteo hacia arriba

  timer_set_period(TIM4, MAX_COUNT - 1); // 72M/2/10000 = 3,6kHz
  timer_set_prescaler(TIM4, 1);

  // Configura el canal PWM para el LED en PB7
  timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1); // PWM1: activo alto
  timer_enable_oc_output(TIM4, TIM_OC4);          // Habilitar salida OC2

  timer_enable_preload(TIM4);
  timer_generate_event(TIM4, TIM_EGR_UG);
  // Activa el contador del timer
  timer_enable_counter(TIM4);
}
void exti_setup()
{
  rcc_periph_clock_enable(RCC_AFIO);  // Habilita el reloj para AFIO
  nvic_enable_irq(NVIC_EXTI4_IRQ);  // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(EXTI4, GPIO4); // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);  // Configura el trigger de la interrupción
  exti_enable_request(EXTI4); // Habilita la solicitud de interrupción

  nvic_enable_irq(NVIC_EXTI3_IRQ);  // Habilita la interrupción del EXTI4 en el NVIC
  exti_select_source(EXTI3, GPIO3); // Selecciona el pin AO4 como fuente de interrupción
  exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH);  // Configura el trigger de la interrupción
  exti_enable_request(EXTI3); // Habilita la solicitud de interrupción
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
}
void configure_dma(uint16_t *adc_bateria)
{
  // Habilita el reloj para el DMA1 y GPIOC
  rcc_periph_clock_enable(RCC_DMA1);

  // Configura el dma para transferir datos
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  // Origen de los datos a transferir:
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, &ADC_DR(ADC1));
  // Dirección de datos destino. El ODR (Output Data Register) del GPIOA:
  dma_set_memory_address(DMA1, DMA_CHANNEL1, adc_bateria);
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
void configure_hardware(){
    systemInit();
    configure_pins();
    configure_systick();
    configure_timer();
    configure_adc();
    configure_usart();
    configure_pwm();
    exti_setup();
}
