#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#define I2C_LCD_ADDR 0x27 //direccion del display

void clock_setup(void){
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
}

void i2c_setup(void){
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO6 | GPIO7);
    i2c_peripheral_disable(I2C1);

    i2c_set_standard_mode(I2C1);
    i2c_set_clock_frequency(I2C1,I2C_CR2_FREQ_42MHZ);
    i2c_set_trise(I2C1,48); //tiempo de subida
    i2c_set_dutycycle(I2C1,I2C_CCR_DUTY_DIV2);
    i2c_set_ccr(I2C1,240);

    i2c_peripheral_enable(I2C1);
}

void lcd_send(uint8_t data, uint8_t mode){
    uint8_t upper= data & 0xF0;
    uint8_t lower= (data<<4) & 0xF0;
    uint8_t data_u = upper | mode | 0x04;
    uint8_t data_l = lower | mode | 0x04;

    i2c_transfer7(I2C1, I2C_LCD_ADDR, &data_u, 1,NULL, 0);
    data_u &= ~0x04;
    i2c_transfer7(I2C1, I2C_LCD_ADDR, &data_u, 1,NULL, 0);

    i2c_transfer7(I2C1, I2C_LCD_ADDR, &data_l, 1,NULL, 0);
    data_l &= ~0x04;
    i2c_transfer7(I2C1, I2C_LCD_ADDR, &data_l, 1,NULL, 0);
}

void lcd_init(void){
    lcd_send(0x03,0);
    lcd_send(0x03,0);
    lcd_send(0x03,0);
    lcd_send(0x02,0);

    lcd_send(0x28,0); //configura LCD en 4 bits, 2 lineas, 5x7
    lcd_send(0x08,0); //display off
    lcd_send(0x01,0); //clear display
    lcd_send(0x06,0); //entry mode
    lcd_send(0x0C,0); //display on
}

void lcd_write_char(char c){
    lcd_send(c,1);
}

void lcd_write_string(const char *str){
    while(*str) lcd_write_char(*str++);
}

int main(void){
    clock_setup();
    i2c_setup();
    lcd_init();
    lcd_write_string("Hello World!");

    while(1){
        //Bucle principal
    }
    return 0;
}
