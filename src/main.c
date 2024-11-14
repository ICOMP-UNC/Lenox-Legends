#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <stdint.h>

#include "lcd_i2c.h"
 
int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Configura el sistema para 72 MHz
    lcd_init();

    volatile int contador=-1;
    char buffer[16];

    while(1){
        contador++;
        if(contador==5){
            lcd_clear();
            sprintf(buffer, "PRUE", contador);
        }
        else{
            lcd_clear();
            sprintf(buffer, "Contador: %d", contador);
        }
        lcd_set_cursor(0, 0);
        lcd_print(buffer);
        delay_ms(1000);
    }
}
