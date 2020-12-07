/* LS027B7DH01 Sharp Memory Display Driver code
 * LS027B7DH01.c
 *
 *  Created on: Nov 26, 2020
 *      Author: TinLethax (Thipok Jiamjarapan)
 *      email : thipok17@gmail.com
 */
#include "LS027B7DH01.h"
#include "font8x8_basic.h"
#include <string.h>


//Display Commands
uint8_t clearCMD[2] = {0x04,0x00}; // Display Clear
uint8_t printCMD[2] = {0x01,0x00}; // Display Bitmap (after issued display update)


//This buffer holds 50 Bytes * 240 Row = 12K of Display buffer
static uint8_t DispBuf[12000];// entire display buffer.

//This buffer holds temporary 2 Command bytes
static uint8_t SendBuf[2];

//This buffer holds 1 Character bitmap image (8x8)
static uint8_t chBuf[8];

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

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDon,GPIO_PIN_SET);// Turn display back on
	//Start 20Hz PWM for COM inversion of the display
	HAL_TIM_PWM_Start(MemDisp->TimerX,MemDisp->COMpwm);
	MemDisp->TimerX->Instance->CCR1 = 5;

	//Clean the Buffer
	memset(DispBuf, 0xFF, 12000);

	//At lease 3 + 13 clock is needed for Display clear (16 Clock = 8x2 bit = 2 byte)
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);
	HAL_SPI_Transmit(MemDisp->Bus, (uint8_t *)clearCMD, 2,150); //According to Datasheet
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);


}

// Display update (Transmit data)
void LCD_Update(LS027B7DH01 *MemDisp){
	SendBuf[0] = printCMD[0]; // M0 High, M2 Low
	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);// Begin

	for(uint8_t count;count < 240;count++){
	SendBuf[1] = count+1;// counting from row number 1 to row number 240
	//row to DispBuf offset
	uint16_t offset = ((count * 50) - 50 <= 0) ? 0 : (count * 50) - 50;

	HAL_SPI_Transmit(MemDisp->Bus, SendBuf, 2, 150);
	HAL_SPI_Transmit(MemDisp->Bus, DispBuf+offset, 50, 150);
	}
	//Send the Dummies bytes after whole display data transmission
	HAL_SPI_Transmit(MemDisp->Bus,(uint8_t *)0x00,2,100);

	HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);// Done
}

//Clean the Buffer
void LCD_BufClean(void){

	memset(DispBuf, 0xFF, 12000);
}

// Clear entire Display
void LCD_Clean(LS027B7DH01 *MemDisp){
		//At lease 3 + 13 clock is needed for Display clear (16 Clock = 8x2 bit = 2 byte)
		HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_SET);
		HAL_SPI_Transmit(MemDisp->Bus, (uint8_t *)clearCMD, 2,150); //According to Datasheet
		HAL_GPIO_WritePin(MemDisp->dispGPIO,MemDisp->LCDcs,GPIO_PIN_RESET);

}

// Buffer update (full 400*240 pixels)
void LCD_LoadFull(uint8_t * BMP[]){
	memcpy(DispBuf, BMP, 12000);
}

// Buffer update (with X,Y Coordinate and image WxH) X,Y Coordinate start at (1,1) to (50,240)
//
//NOTE THAT THE X COOR and WIDTH ARE BYTE NUMBER NOT PIXEL NUMBER (8 pixel = 1 byte). A.K.A IT'S BYTE ALIGNED
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

/* FIXME TODO */
//Similar to LCD_LoadPart, but x,y coordinate are BOTH PIXEL position.
void LCD_LoadPix(uint8_t* BMP[], uint16_t Xcord, uint8_t Ycord, uint16_t bmpW, uint8_t bmpH){
	if ((bmpW > 400) | (Xcord >400) | (Ycord > 240) | (bmpH > 240)) return;

	Xcord = Xcord - 1;
	Ycord = Ycord - 1;
	bmpW = bmpW - 1;
	bmpH = bmpH - 1;

	bmpW = (uint8_t)(bmpW / 8);

	//Shifting value to align the pixel
	uint8_t Shiftval = (uint8_t)(Xcord % 8);

	//Counting from Y origin point to bmpH using for loop
	for(uint8_t loop = 0; loop < bmpH; loop++){
		// turn X an Y into absolute offset number for Buffer
		uint16_t XYoff = ( (Ycord + loop == 0) ? 0 : (( (Ycord+loop) * 50 ) - 50) )  + (uint16_t)(Xcord/8);

		// turn W and H into absolute offset number for Bitmap image
		uint16_t WHoff = ( (loop + 1)*bmpW )- bmpW;

		for (uint16_t i=0;i < bmpW;i ++){
			DispBuf[i+XYoff] = (uint8_t)(*BMP[WHoff + i] >> Shiftval);
			DispBuf[i+XYoff+1] = (uint8_t)(*BMP[WHoff + i] << (7 - Shiftval));
		}

	}
}


//Invert color of Display memory buffer
void LCD_Invert(void){
	uint16_t invt = 12000;
	do{
	DispBuf[invt-1] = ~DispBuf[invt-1];
	invt--;
	}while(invt);
}

//Fill screen with either black or white color
void LCD_Fill(bool fill){
	memset(DispBuf, (fill ? 0 : 0xFF) , 12000);
}


//Print 8x8 Text on screen, TODO Shorten the code
void LCD_Print(char txtBuf[]){

uint16_t strLen = sizeof(*txtBuf);
uint8_t YLine = 1;
uint8_t Xcol = 1;
uint8_t chOff = 0;

for (uint16_t p = 0; p < strLen;p++){
	// In case of reached 50 chars or newline detected , Do the newline
	if ((Xcol > 50) || *txtBuf == 0x0A){
		Xcol = 1;// Move cursor to most left
		YLine += 8;// enter new line
		txtBuf++;// move to next char
	}

	// Avoid printing Newline
	if (*txtBuf != 0x0A){

	chOff = (*txtBuf - 0x20) ? 0 : ( (*txtBuf - 0x20) * 8) - 8 ;// calculate char offset (fist 8 pixel of character)

	for(uint8_t i=0;i < 8;i++){// Copy the inverted color px to buffer
	chBuf[i] = ~font8x8_basic[i + chOff];
	}

	LCD_LoadPart((uint8_t **)chBuf, Xcol, YLine, 1, 8);// Align the char with the 8n pixels

	txtBuf++;// move to next char
	Xcol++;// move cursor to next column
	}
  }
}
