
#include <stdint.h>
// Control del LCD
#define LCD_BACKLIGHT 0x08 // Backlight ON
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

#define I2C_LCD_ADDR 0x27 // Cambia a 0x3F si tu adaptador usa esa dirección
#define I2C_BUS I2C1

void lcd_init(void);
void lcd_clear(void);
void lcd_print(const char *str);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print_int(int value);
void delay_ms(uint32_t ms);