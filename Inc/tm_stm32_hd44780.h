/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   HD44780 LCD driver library for STM32Fxxx
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef AT_HD44780_H
#define AT_HD44780_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup AT_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup AT_HD44780
 * @brief    HD44780 LCD driver library for STM32Fxxx - http://stm32f4-discovery.net/2015/07/hal-library-15-hd44780-for-stm32fxxx/
 * @{
 *
 *	It also supports all HD44780 compatible LCD drivers.
 *
 * \par Default pinout
 *	
\verbatim
LCD   STM32Fxxx         DESCRIPTION

GND   GND               Ground
VCC   +5V               Power supply for LCD
V0    Potentiometer	    Contrast voltage. Connect to potentiometer
RS    PB2               Register select, can be overwritten in your project's defines.h file
RW    GND               Read/write
E     PB7               Enable pin, can be overwritten in your project's defines.h file
D0    -                 Data 0 - doesn't care
D1    -                 Data 1 - doesn't care
D2    -                 Data 2 - doesn't care
D3    -                 Data 3 - doesn't  care
D4    PC12              Data 4, can be overwritten in your project's defines.h file
D5    PC13              Data 5, can be overwritten in your project's defines.h file
D6    PB12              Data 6, can be overwritten in your project's defines.h file
D7    PB13              Data 7, can be overwritten in your project's defines.h file
A     +3V3              Back light positive power
K     GND               Ground for back light
\endverbatim	
 *	
 * If you want to change pinout, do this in your defines.h file with lines below and set your own settings:
 *	
\code
//RS - Register select pin
#define HD44780_RS_PORT     GPIOB
#define HD44780_RS_PIN      GPIO_PIN_2
//E - Enable pin
#define HD44780_E_PORT      GPIOB
#define HD44780_E_PIN       GPIO_PIN_7
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOC
#define HD44780_D4_PIN      GPIO_PIN_12
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOC
#define HD44780_D5_PIN      GPIO_PIN_13
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOB
#define HD44780_D6_PIN      GPIO_PIN_12
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOB
#define HD44780_D7_PIN      GPIO_PIN_13
\endcode
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - STM32Fxxx HAL
 - defines.h
 - TM DELAY
 - TM GPIO
\endverbatim
 */
//#include "stm32fxxx_hal.h"
//#include "defines.h"
//#include "AT_stm32_delay.h"
//#include "AT_stm32_gpio.h"
  #include "stm32f4xx_hal.h"
	#include "delay_micros.h"

/**
 * @defgroup AT_HD44780_Macros
 * @brief    Library defines
 * @{
 */

const unsigned char russian_char[]={ 0x41, 0xA0, 0x42, 0xA1, 0xE0, 0x45,
0xA3, 0xA4, 0xA5,0xA6, 0x4B, 0xA7, 0x4D, 0x48, 0x4F, 0xA8, 0x50,0x43,
0x54, 0xA9, 0xAA, 0x58, 0xE1, 0xAB, 0xAC, 0xE2, 0xAD,0xAE, 0x62,
0xAF, 0xB0, 0xB1, 0x61, 0xB2, 0xB3, 0xB4, 0xE3, 0x65, 0xB6, 0xB7,
0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0x6F, 0xBE, 0x70, 0x63,0xBF,
0x79, 0xE4, 0x78, 0xE5, 0xC0, 0xC1, 0xE6, 0xC2,0xC3, 0xC4, 0xC5,
0xC6, 0xC7};



/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOE
#define HD44780_RS_PIN				GPIO_PIN_14
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOE
#define HD44780_E_PIN				GPIO_PIN_15
#endif
/* Data pins */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOB
#define HD44780_D4_PIN				GPIO_PIN_12
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOB
#define HD44780_D5_PIN				GPIO_PIN_13
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOB
#define HD44780_D6_PIN				GPIO_PIN_14
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOB
#define HD44780_D7_PIN				GPIO_PIN_15
#endif

/**
 * @}
 */

/**
 * @defgroup AT_HD44780_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes HD44780 LCD
 * @brief  cols: Width of lcd
 * @param  rows: Height of lcd
 * @retval None
 */
void AT_HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void AT_HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void AT_HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void AT_HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x: X location where string will start
 * @param  y; Y location where string will start
 * @param  *str: pointer to string to display
 * @retval None
 */
void AT_HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void AT_HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void AT_HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void AT_HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void AT_HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void AT_HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void AT_HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void AT_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  x: X location where character will be shown
 * @param  y: Y location where character will be shown
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
void AT_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif