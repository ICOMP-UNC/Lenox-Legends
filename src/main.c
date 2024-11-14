#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <stdint.h>

#define I2C_LCD_ADDR 0x27 // Cambia a 0x3F si tu adaptador usa esa dirección
#define I2C_BUS I2C1

// Control del LCD
#define LCD_BACKLIGHT 0x08 // Backlight ON
#define LCD_ENABLE    0x04
#define LCD_RS        0x01


void delay_ms(uint32_t ms){
    for(int i=0; i<ms*8000; i++){
        __asm__("nop");
    }
}
void i2c_write(uint8_t addr, uint8_t data) {
    i2c_transfer7(I2C_BUS, addr, &data, 1, NULL, 0);
}

void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble << 4) | LCD_BACKLIGHT | mode;
    i2c_write(I2C_LCD_ADDR, data | LCD_ENABLE); // Enable pulse
    i2c_write(I2C_LCD_ADDR, data);              // Clear enable
}

void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte >> 4, mode); // Parte alta
    lcd_send_nibble(byte & 0x0F, mode); // Parte baja
}

void lcd_command(uint8_t command) {
    lcd_send_byte(command, 0);
}

void lcd_data(uint8_t data) {
    lcd_send_byte(data, LCD_RS);
}

void lcd_init(void) {
/*************  ✨ Codeium Command ⭐  *************/
/**
 * @brief Initialize the I2C and GPIO peripherals for LCD communication.
 *
 * This function enables the clock for the GPIOB and I2C1 peripherals,
 * and configures the GPIO pins for open-drain output mode, which is
 * required for I2C communication with the LCD.
 */
/******  ec349dd4-930f-49de-bccb-e9ed1299292b  *******/    // Configura los relojes de GPIOB e I2C1
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Configura los pines de I2C (SCL y SDA en GPIOB6 y GPIOB7)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    // Configura el periférico I2C
    i2c_peripheral_disable(I2C_BUS);             // Desactiva I2C antes de la configuración
    i2c_set_clock_frequency(I2C_BUS, I2C_CR2_FREQ_36MHZ); // Reloj del sistema de 36 MHz
    i2c_set_standard_mode(I2C_BUS);              // Modo estándar (100 kHz)
    i2c_set_ccr(I2C_BUS, 180);                   // Calculado para 100 kHz con 36 MHz de reloj
    i2c_set_trise(I2C_BUS, 37);                  // Tiempo de subida para 100 kHz en 36 MHz
    i2c_peripheral_enable(I2C_BUS);              // Activa I2C
    delay_ms(5);
    // Secuencia de inicialización del LCD
    lcd_send_nibble(0x03, 0); // Inicio en modo de 8 bits7
    delay_ms(5);
    lcd_send_nibble(0x03, 0);
    delay_ms(5);
    lcd_send_nibble(0x03, 0);
    delay_ms(5);
    lcd_send_nibble(0x02, 0); // Cambia a modo de 4 bits
    delay_ms(5);

    // Configuración del LCD en modo 4 bits, 2 líneas, y texto de 5x8 puntos
    lcd_command(0x28); // Modo de 4 bits, 2 líneas
    delay_ms(5);
    lcd_command(0x08); // Apagar display
    delay_ms(5);
    lcd_command(0x01); // Limpiar display
    delay_ms(5);
    lcd_command(0x06); // Configuración de entrada
    delay_ms(5);
    lcd_command(0x0C); // Encender display, sin cursor
    delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0 ? 0x00 : 0x40) + col;
    lcd_command(0x80 | address);
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data((uint8_t)(*str));
        str++;
    }
}

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Configura el sistema para 72 MHz
    lcd_init();

    volatile int contador=-1;
    char buffer[16];

    while(1){
        contador++;
        if(contador==5){
            sprintf(buffer, "PRUEBA", contador);
        }
        else{
            sprintf(buffer, "Contador: %d", contador);
        }
        lcd_set_cursor(0, 0);
        lcd_print(buffer);
        delay_ms(1000);
    }
}
