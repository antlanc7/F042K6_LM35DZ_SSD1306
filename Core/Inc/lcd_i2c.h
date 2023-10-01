/*
 * lcd_i2c.h
 *
 *  Created on: 5 dic 2022
 *      Author: anton
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "lcd_i2c_conf.h"

void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_send_string_clear_rest_line(char *str);
void lcd_clear (void);


#endif /* INC_LCD_I2C_H_ */
