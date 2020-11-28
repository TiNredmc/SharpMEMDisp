# SharpMEMDisp
Sharp LS027B7DH01 controller code for STM32F3 Discovery board.

# Available Command

1. LCD_Init // Display Init
2. LCD_Clean // Display Clear
3. LCD_LoadFull // Load Full Screen data onto buffer mem 
4. LCD_LoadPart // load specific bitmap to buffer mem with X,Y coordinate (please read more in the source)
5. LCD_Invert // Invert Color of all pixels in buffer mem
6. LCD_Update // Transmit buffer to display memory