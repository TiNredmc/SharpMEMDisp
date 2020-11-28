/* LS027B7DH01 Sharp Memory Display Driver code
 * LS027B7DH01.c
 *
 *  Created on: Nov 26, 2020
 *      Author: TinLethax
 */
#include "LS027B7DH01.h"
#include <string.h>

//Display Commands
uint8_t clearCMD[2] = {0x04,0x00}; // Display Clear
uint8_t printCMD[2] = {0x01,0x00}; // Display Bitmap (after issued display update)


// Display Initialization
void LCD_Init(LS027B7DH01 *MemDisp, SPI_HandleTypeDef *Bus,
		GPIO_TypeDef *dispGPIO,uint16_t LCDcs,uint16_t LCDon,
		TIM_HandleTypeDef *TimerX, uint32_t COMpwm){

	//Store params into our struct
	MemDisp->Bus = Bus;
	MemDisp->dispGPIO = dispGPIO;
	MemDisp->TimerX = TimerX;
	MemDisp->COMpwm = COMpwm;
	MemDisp->LCDcs = LCDcs;
	MemDisp->LCDon = LCDon;

	//Start 20Hz PWM for COM inversion of the display
	HAL_TIM_PWM_Start(MemDisp->TimerX,MemDisp->COMpwm);
	MemDisp->TimerX->Instance->CCR1 = 5;

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_RESET);// Turn display off (Temp)
	//At lease 3 + 13 clock is needed for Display clear (16 Clock = 8x2 bit = 2 byte)
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);
	HAL_SPI_Transmit(MemDisp->Bus, (uint8_t *)clearCMD, 2,100); //According to Datasheet
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_SET);// Turn display back on
}

// Display update (Transmit data)
void LCD_Update(LS027B7DH01 *MemDisp){
	SendBuf[0] = 0x01; // M0 High, M2 Low
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_RESET);// Turn display off (Temp)
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);// Begin

	for(uint8_t count;count < 240;count++){
	SendBuf[1] = count+1;// counting from row number 1 to row number 240
	uint16_t offset = (count == 0) ? 0 : (count * 50) - 50;

		for(uint8_t arrCnt ; arrCnt < 50; arrCnt++){// passing data from display buffer to transmit buffer.
			SendBuf[arrCnt+2] = DispBuf[arrCnt+offset];
		}

	HAL_SPI_Transmit(MemDisp->Bus,(uint8_t *)SendBuf,52,100);
	}
	//Send the Dummies buytes after whole display data transmission
	HAL_SPI_Transmit(MemDisp->Bus,(uint8_t *)0x00,2,100);

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);// Done

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);// Begin
	HAL_SPI_Transmit(MemDisp->Bus, (uint8_t *)printCMD,2,100 );
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);// Done

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_SET);// Turn display back on
}

// Clear entire Display
void LCD_Clean(LS027B7DH01 *MemDisp){
	   HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_RESET);// Turn display off (Temp)
		//At lease 3 + 13 clock is needed for Display clear (16 Clock = 8x2 bit = 2 byte)
		HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);
		HAL_SPI_Transmit(MemDisp->Bus, (uint8_t *)clearCMD, 2,100); //According to Datasheet
		HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);

		HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_SET);// Turn display back on
}

// Buffer update (full 400*240 pixels)
void LCD_LoadFull(uint8_t * BMP[]){
	memcpy(DispBuf, BMP, 12000);
}

// Buffer update (with X,Y Coordinate and image WxH) X,Y Coordinate start at (1,1) to (50,240)
//
//NOTE THAT THE X COOR and WIDTH ARE BYTE NUMBER NOT PIXEL NUMBER (8 pixel = 1 byte). A.K.A IT'S PIXEL ALIGNED
//
void LCD_LoadPart(uint8_t* BMP[], uint8_t Xcord, uint8_t Ycord, uint8_t bmpW, uint8_t bmpH){
	if ((bmpW > 50) | (Xcord >50) | (Ycord > 240) | (bmpH > 240)) return;

	Xcord = Xcord - 1;
	Ycord = Ycord - 1;
	bmpW = bmpW - 1;
	bmpH = bmpH - 1;

	//Counting from Y origin point to bmpH using for loop
	for(uint8_t loop = 0; loop < bmpH; loop++){
		// turn X an Y into absolute offset number for Buffer
		uint16_t XYoff = ( (Ycord + loop == 0) ? 0 : (( (Ycord+loop) * 50 ) - 50) )   +    Xcord;

		// turn W and H into absolute offset number for Bitmap image
		uint16_t WHoff = ( (loop + 1)*bmpW )- bmpW;

		memcpy(DispBuf + XYoff, BMP+WHoff, bmpW);
	}

}




