/*
 * lcd_i2c_conf.h
 *
 *  Created on: 5 dic 2022
 *      Author: anton
 */

#ifndef INC_LCD_I2C_CONF_H_
#define INC_LCD_I2C_CONF_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define LCD_I2C_ADDRESS (0x27<<1)
#define LCD_I2C_HANDLE &hi2c1

#endif /* INC_LCD_I2C_CONF_H_ */
