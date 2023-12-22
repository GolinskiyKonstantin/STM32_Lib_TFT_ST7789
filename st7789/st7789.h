/*
  ******************************************************************************
  * @file 			( фаил ):   ST7789.h
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	 author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
 */
 
 
#ifndef _ST7789_H
#define _ST7789_H


/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

// Обязательно нужен #include "main.h" 
// чтоб отдельно не подключать файлы связанные с МК и стандартными библиотеками

#include "main.h"
#include "fonts.h"

#include "stdlib.h"
#include "string.h"
#include "math.h"



//#######  SETUP  ##############################################################################################
		
		//==== выбераем через что будем отправлять через HAL или CMSIS(быстрее) ==================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
			// указываем порт SPI для CMSIS ( быстро )-------
			// так как у разных МК разные регистры то в функциях корректируем под свой МК
			// на данный момент есть реализация на серию F1 F4 H7 для выбора серии в функциях
			//	void ST7789_SendCmd(uint8_t Cmd);
			//	void ST7789_SendData(uint8_t Data );
			//	void ST7789_SendDataMASS(uint8_t* buff, size_t buff_size);	
			// комментируем и раскомментируем то что нам нужно, также там же редактируем под свой МК если не работает
			//#define 	ST7789_SPI_CMSIS 	SPI2
			//-----------------------------------------------
			
			// указываем порт SPI для HAL ( медлено )--------
			#define 	ST7789_SPI_HAL 		hspi1
			//-----------------------------------------------
			
		//============================================================================
			
			// выбираем как выводить информацию через буфер кадра или попиксельно ( 1-буфер кадра, 0-попиксельный вывод ) -----
			// через буфер быстре если много информации обнавлять за один раз ( требует много оперативки для массива )
			// по пиксельно рисует онлайн буз буферра если информация обновляеться немного то выгодно испотзовать данный режим
			#define FRAME_BUFFER				0
			//-----------------------------------------------------------------------------------------------------------------
			
			
		//=== указываем порты ( если в кубе назвали их DC RES CS то тогда нечего указывать не нужно )
		#if defined (DC_GPIO_Port)
		#else
			#define DC_GPIO_Port	GPIOC
			#define DC_Pin			GPIO_PIN_5
		#endif
		
		#if defined (RST_GPIO_Port)
		#else
			#define RST_GPIO_Port   GPIOB
			#define RST_Pin			GPIO_PIN_14
		#endif
		
		//--  Cесли используем порт CS для выбора устройства тогда раскомментировать ------------
		// если у нас одно устройство лучше пин CS притянуть к земле( или на порту подать GND )
		
		#define CS_PORT
		
		//----------------------------------------------------------------------------------------
		#ifdef CS_PORT
			#if defined (CS_GPIO_Port)
			#else
				#define CS_GPIO_Port    GPIOB
				#define CS_Pin			GPIO_PIN_12
			#endif
		#endif
		
		//=============================================================================
		
		//==  выбираем дисплей: =======================================================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
		//#define	ST7789_IS_135X240		// 1.14" 135 x 240 ST7789 
		#define	ST7789_IS_240X240		// 1.3" 240 x 240 ST7789 
		//#define	ST7789_IS_172X320		// 1.47" 172 x 320 ST7789 
		//#define	ST7789_IS_240X320		// 2" 240 x 320 ST7789
		
		//=============================================================================
		
		
//##############################################################################################################

#ifdef ST7789_SPI_HAL
	extern SPI_HandleTypeDef ST7789_SPI_HAL;
#endif

extern uint16_t ST7789_Width, ST7789_Height;

extern uint16_t ST7789_X_Start;
extern uint16_t ST7789_Y_Start;

#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define PI 	3.14159265

//--- готовые цвета ------------------------------
#define   	ST7789_BLACK   			0x0000
#define   	ST7789_BLUE    			0x001F
#define   	ST7789_RED     			0xF800
#define   	ST7789_GREEN   			0x07E0
#define 		ST7789_CYAN    			0x07FF
#define 		ST7789_MAGENTA 			0xF81F
#define 		ST7789_YELLOW  			0xFFE0
#define 		ST7789_WHITE   			0xFFFF
//------------------------------------------------

//-- Битовые маски настройки цветности ST7789 ----
#define ST7789_ColorMode_65K    	0x50
#define ST7789_ColorMode_262K   	0x60
#define ST7789_ColorMode_12bit  	0x03
#define ST7789_ColorMode_16bit  	0x05
#define ST7789_ColorMode_18bit  	0x06
#define ST7789_ColorMode_16M    	0x07
//------------------------------------------------

#define ST7789_MADCTL_MY  				0x80
#define ST7789_MADCTL_MX  				0x40
#define ST7789_MADCTL_MV  				0x20
#define ST7789_MADCTL_ML  				0x10
#define ST7789_MADCTL_RGB 				0x00
#define ST7789_MADCTL_BGR 				0x08
#define ST7789_MADCTL_MH  				0x04
//-------------------------------------------------


#define ST7789_SWRESET 						0x01
#define ST7789_SLPIN   						0x10
#define ST7789_SLPOUT  						0x11
#define ST7789_NORON   						0x13
#define ST7789_INVOFF  						0x20
#define ST7789_INVON   						0x21
#define ST7789_DISPOFF 						0x28
#define ST7789_DISPON  						0x29
#define ST7789_CASET   						0x2A
#define ST7789_RASET   						0x2B
#define ST7789_RAMWR   						0x2C
#define ST7789_COLMOD  						0x3A
#define ST7789_MADCTL  						0x36
//-----------------------------------------------

#define DELAY 										0x80


//###  параметры дисплея 1.3" 240 x 240 ST7789 ###################################

	// 1.3" 240 x 240 ST7789  display, default orientation

#ifdef ST7789_IS_240X240

	#define ST7789_WIDTH  			240
	#define ST7789_HEIGHT 			240
	#define ST7789_XSTART 			0
	#define ST7789_YSTART 			0
	#define ST7789_ROTATION 		(ST7789_MADCTL_RGB)
	
#endif
	
//##############################################################################


//###  параметры дисплея 1.14" 135 x 240 ST7789 ###################################

	// 1.14" 135 x 240 ST7789  display, default orientation

#ifdef ST7789_IS_135X240

	#define ST7789_WIDTH  			135
	#define ST7789_HEIGHT 			240
	#define ST7789_XSTART 			52
	#define ST7789_YSTART 			40
	#define ST7789_ROTATION 		(ST7789_MADCTL_RGB)
	
#endif
	
//##############################################################################


//##############################################################################


//###  параметры дисплея 1.47" 172 x 320 ST7789 ###################################

	// 1.47" 172 x 320 ST7789 display, default orientation
		
#ifdef ST7789_IS_172X320
	
	#define ST7789_WIDTH  			320
	#define ST7789_HEIGHT 			172
	#define ST7789_XSTART 			0
	#define ST7789_YSTART 			34
	#define ST7789_ROTATION 		(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB)
	
#endif
	
//##############################################################################


//###  параметры дисплея 2" 240 x 320 ST7789 ###################################

	// 2" 240 x 320 ST7789  display, default orientation

#ifdef ST7789_IS_240X320

	#define ST7789_WIDTH  			240
	#define ST7789_HEIGHT 			320
	#define ST7789_XSTART 			0
	#define ST7789_YSTART 			0
	#define ST7789_ROTATION 		(ST7789_MADCTL_RGB)
	
#endif
	
//##############################################################################

void ST7789_Init(void);
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);	
void ST7789_HardReset(void);
void ST7789_SleepModeEnter( void );
void ST7789_SleepModeExit( void );
void ST7789_ColorModeSet(uint8_t ColorMode);
void ST7789_MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror, uint8_t HorizMirror, uint8_t IsBGR);
void ST7789_InversionMode(uint8_t Mode);
void ST7789_FillScreen(uint16_t color);
void ST7789_Clear(void);
void ST7789_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7789_SetBL(uint8_t Value);
void ST7789_DisplayPower(uint8_t On);
void ST7789_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7789_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor);
void ST7789_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7789_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color);
void ST7789_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ST7789_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor);
void ST7789_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
void ST7789_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void ST7789_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void ST7789_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void ST7789_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void ST7789_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch);
void ST7789_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch);
void ST7789_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str);
void ST7789_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str);
void ST7789_rotation( uint8_t rotation );
void ST7789_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void ST7789_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees);
void ST7789_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color);
void ST7789_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void ST7789_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void ST7789_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void ST7789_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick);
void ST7789_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick);
void ST7789_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick);

#if FRAME_BUFFER
	void ST7789_Update(void);
	void ST7789_ClearFrameBuffer(void);
#endif


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif	/*	_ST7789_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
