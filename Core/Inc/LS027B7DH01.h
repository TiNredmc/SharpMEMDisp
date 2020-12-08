/* LS027B7DH01 Sharp Memory Display Driver code
 * LS027B7DH01.h
 *
 *  Created on: Nov 26, 2020
 *      Author: TinLethax (Thipok Jiamjarapan)
 *     	email : thipok17@gmail.com
 */

#ifndef INC_LS027B7DH01_H_
#define INC_LS027B7DH01_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>

// This typedef holds the hardware parameters. For GPIO and SPI
typedef struct {
	SPI_HandleTypeDef 	*Bus;
	GPIO_TypeDef 		*dispGPIO;
	TIM_HandleTypeDef	*TimerX;
	uint32_t			 COMpwm;
	uint16_t 			 LCDcs;
	uint16_t			 LCDon;
}LS027B7DH01;


void LCD_Init(LS027B7DH01 *MemDisp, SPI_HandleTypeDef *Bus,
		GPIO_TypeDef *dispGPIO,uint16_t LCDcs,uint16_t LCDon,
		TIM_HandleTypeDef *TimerX, uint32_t COMpwm);
void LCD_Clean(LS027B7DH01 *MemDisp);
void LCD_Update(LS027B7DH01 *MemDisp);
void LCD_LoadFull(uint8_t * BMP);
void LCD_LoadPart(uint8_t* BMP, uint8_t Xcord, uint8_t Ycord, uint8_t bmpW, uint8_t bmpH);
void LCD_LoadPix(uint8_t* BMP, uint16_t Xcord, uint8_t Ycord, uint16_t bmpW, uint8_t bmpH);
void LCD_Print(char txtBuf[],size_t len);
void LCD_BufClean(void);
void LCD_Invert(void);
void LCD_Fill(bool fill);

#endif /* INC_LS027B7DH01_H_ */
