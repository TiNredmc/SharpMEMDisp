# SharpMEMDisp
Sharp LS027B7DH01 controller code for STM32F3 Discovery board.

# Available Command

1. LCD_Init // Display Init
2. LCD_Clean // Display Clear
3. LCD_LoadFull // Load Full Screen data onto buffer mem 
4. LCD_LoadPart // load specific bitmap to buffer mem with X,Y coordinate (Byte aligned)
5. LCD_LoadPix // load specific bitmap to buffer mem with X,Y coordinate (Pixel aligned)
6. LCD_Print // Print string with 8x8 font
7. LCD_Invert // Invert Color of all pixels in buffer mem
8. LCD_BufClean // Set all byte in buffer to 0xFF (appear as white on Display when LCD_Update)
9. LCD_Fill // true : fill with black, false : fill with white
10. LCD_Update // Transmit buffer to display memory
