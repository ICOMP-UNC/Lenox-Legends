#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

int main(void){
    //se configura el clock del sistema a 72Mhz
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    //se configura el led
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    while(1){
        for(int i=0;i<1000000;i++){
            __asm__("nop");
            gpio_toggle(GPIOC, GPIO13);
        }
    }
}