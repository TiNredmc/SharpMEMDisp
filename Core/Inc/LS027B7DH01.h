/*
 * LS027B7DH01.h
 *
 *  Created on: Nov 26, 2020
 *      Author: TinLethax
 */

#ifndef INC_LS027B7DH01_H_
#define INC_LS027B7DH01_H_

#include "stm32f3xx_hal.h"

//This buffer holds 50 Bytes * 240 Row = 12K of Display buffer
uint8_t DispBuf[12000];// entire display buffer.

//This buffer holds temporary 52 Bytes including 2 Command bytes and 50 Pixel Bytes (50x8bit = 400 pixel)
uint8_t SendBuf[52];

// This typedef holds the hardware parameters. For GPIO and SPI
typedef struct {
	SPI_HandleTypeDef 	*Bus;
	GPIO_TypeDef 		*dispGPIO;
	TIM_HandleTypeDef			*TimerX;
	uint32_t			 COMpwm;
	uint16_t 			 LCDcs;
	uint16_t			 LCDon;
}LS027B7DH01;


void LCD_Init(LS027B7DH01 *MemDisp, SPI_HandleTypeDef *Bus,
		GPIO_TypeDef *dispGPIO,uint16_t LCDcs,uint16_t LCDon,
		TIM_HandleTypeDef *TimerX, uint32_t COMpwm);
void LCD_Clean(LS027B7DH01 *MemDisp);
void LCD_Update(LS027B7DH01 *MemDisp);
void LCD_LoadFull(uint8_t * BMP[]);
void LCD_LoadPart(uint8_t* BMP[], uint8_t Xcord, uint8_t Ycord, uint8_t bmpW, uint8_t bmpH);

#endif /* INC_LS027B7DH01_H_ */