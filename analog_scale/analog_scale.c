/*

  ******************************************************************************
  * @file 			( фаил ):   analog_scale.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

/* Includes ----------------------------------------------------------*/
#include "analog_scale.h"

#include "math.h"
#include "stdio.h"
#include "ST7789.h"
#include "fonts.h"

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};

int old_analog =  -999; // Value last displayed

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
	
/*
	******************************************************************************
	* @brief	 ( описание ):  
	* @param	( параметры ):	
	* @return  ( возвращает ):	

	******************************************************************************
*/
static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//----------------------------------------------------------------------------------

// #########################################################################
// Обновление данных стрелки и значения на цифербладе ( функцию вызываем с задержкой не менее 10 мс )
// первый параметр передаем наше значение ( от -10 до 110 гнаницы )
// чтоб попадал в диапазон разметки на циферблате передаем от 0 до 100
// второй переметр время для следущего обновления, рекомендуеться ставить 0 и делать вызов функции с интервалом не менее 10 мс
// #########################################################################
void plotNeedle(int8_t value, uint8_t ms_delay)
{
  // формеруем и печатаем текущее значение в левом нижнем углу циферблата
  char buff_value[5];	// ставим на +1 чем нужно для \0
  sprintf( buff_value, "% 4d", value );	// 4 знакоместа выравнивание по правому краю
  ST7789_print( 10, 119 - 20, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, buff_value );
  
  // проверяем границы входных данных
  if (value < -10){
	  value = -10; // Limit value to emulate needle end stops
  }
  if (value > 110){
	  value = 110;
  }
  
  // Move the needle util new value reached
  while (!(value == old_analog)) {
    if (old_analog < value){
		old_analog++;
	}
    else{
		old_analog--;
	}

    if (ms_delay == 0){
		old_analog = value; // Update immediately id delay is 0
	}

    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calcualte tip of needle coords
    float sx = cos((double)sdeg * 0.0174532925);
    float sy = sin((double)sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((double)(sdeg + 90) * 0.0174532925);

    // Erase old needle image
    ST7789_DrawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, RGB565(255, 255, 255));
    ST7789_DrawLine(120 + 20 * ltx, 140 - 20, osx, osy, RGB565(255, 255, 255));
    ST7789_DrawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, RGB565(255, 255, 255));

    // Re-plot text under needle
	ST7789_print( 104, 70, RGB565(200, 200, 200) , RGB565(255, 255, 255) , 1, &Font_11x18, 1, "%RH" ); // надпись по середине индикатора

    // Store new needle end coords for next erase
    ltx = tx;
    osx = sx * 98 + 120;
    osy = sy * 98 + 140;

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
	// рисуем стрелку циферблата
    ST7789_DrawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, RGB565(255, 0, 0));
    ST7789_DrawLine(120 + 20 * ltx, 140 - 20, osx, osy, RGB565(255, 50, 50));
    ST7789_DrawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, RGB565(255, 0, 0));

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10){
		ms_delay += ms_delay / 5;
	}

    // Wait before next update
    HAL_Delay (ms_delay);
  }
}

// #########################################################################
// рисуем сам циферблат ( вызываем функцию 1 раз в начале )
// #########################################################################
void analogMeter(void)	
{
  // размер всего окна 240 на 126	
  // Meter outline
  ST7789_FillRect(0, 0, 240, 126, RGB565(80, 80, 80) );	// первичная ( внешняя ) рамка циферблата
  ST7789_FillRect(5, 3, 230, 120, RGB565(255, 255, 255) );	// внутренее поле циферблата

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (100 + tl) + 120;
    uint16_t y0 = sy * (100 + tl) + 140;
    uint16_t x1 = sx * 100 + 120;
    uint16_t y1 = sy * 100 + 140;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (100 + tl) + 120;
    int y2 = sy2 * (100 + tl) + 140;
    int x3 = sx2 * 100 + 120;
    int y3 = sy2 * 100 + 140;

    // Yellow zone limits
    //if (i >= -50 && i < 0) {
    //  ST7789_DrawFilledTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
    //  ST7789_DrawFilledTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    //}

    // Green zone limits	зона зеленая
    if (i >= 0 && i < 25) {
      ST7789_DrawFilledTriangle(x0, y0, x1, y1, x2, y2, RGB565(100, 255, 100) );
      ST7789_DrawFilledTriangle(x1, y1, x2, y2, x3, y3, RGB565(100, 255, 100) );
    }

    // Orange zone limits	зона красная
    if (i >= 25 && i < 50) {
      ST7789_DrawFilledTriangle(x0, y0, x1, y1, x2, y2, RGB565(255, 100, 100) );
      ST7789_DrawFilledTriangle(x1, y1, x2, y2, x3, y3, RGB565(255, 100, 100) );
    }

    // Short scale tick length
    if (i % 25 != 0){
		tl = 8;
	}
	
    // Recalculate coords incase tick lenght changed
    x0 = sx * (100 + tl) + 120;
    y0 = sy * (100 + tl) + 140;
    x1 = sx * 100 + 120;
    y1 = sy * 100 + 140;

    // Draw tick линии и цвет делений между значениями циферблата
    ST7789_DrawLine(x0, y0, x1, y1, RGB565(0, 0, 0) );

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (100 + tl + 10) + 120;
      y0 = sy * (100 + tl + 10) + 140;
	
	  // печать цифр индикатора
      switch (i / 25) {
        case -2:  
			ST7789_print( x0-9, y0 - 10, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "0" );
			break;
        case -1:
			ST7789_print( x0-8, y0 - 7, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "25" );
			break;
        case 0: 
			ST7789_print( x0-7, y0 - 4, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "50" );
			break;
        case 1: 
			ST7789_print( x0-8, y0 - 7, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "75" );
			break;
        case 2: 
			ST7789_print( x0-9, y0 - 10, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "100" );
			break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * 100 + 120;
    y0 = sy * 100 + 140;
	
    // Draw scale arc, don't draw the last part
    if (i < 50){
		ST7789_DrawLine(x0, y0, x1, y1, RGB565(0, 0, 0) );	// рисуем дугу циферблата
	}
  }
	
  ST7789_print( 5 + 230 - 40, 119 - 20, RGB565(50, 150, 50) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "%RH" );	// надпись в провам нижнем углу
  ST7789_DrawRectangle(6, 4, 233, 121, RGB565(0, 0, 0) ); // тоненькая рамка индикатора

  plotNeedle(0, 0); // Put meter needle at 0
}






// #########################################################################
//  рисуем вертикальную шкалу ( значение для показа от 0 до 100 )
//  первый параметр название шкалы ( например "A1" )
//  второй параметр координата х ( ширина шкалы 40 ) тоесть следующую шкалу рисуем х + 40
//  третий параметр координата у
// #########################################################################
void plotLinear(char *label, int x, int y)
{
  int w = 36;
  ST7789_DrawRectangle(x, y, x + w, y + 155, RGB565(150, 150, 150));	// внешняя тонкая рамка
  ST7789_FillRect(x + 2, y + 19, w - 3, 155 - 38, RGB565(255, 255, 255));	// фон самого индикатора
  ST7789_print( x + w / 2 - 15, y + 3, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, label );	// вывод надписи над индикатором
  
  // рисуем короткие линии на индикаторе
  for (int i = 0; i < 110; i += 10)
  {
    ST7789_DrawLine(x + 20, y + 27 + i, x + 20 + 5, y + 27 + i, RGB565(0, 0, 0));
  }
  // рисуем длинные линии на индикаторе
  for (int i = 0; i < 110; i += 50)
  {
    ST7789_DrawLine(x + 20, y + 27 + i, x + 20 + 11, y + 27 + i, RGB565(0, 0, 0));
  }
  
  // рисуем стрелки индикатора
  ST7789_DrawFilledTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 - 5, RGB565(255, 0, 0) );
  ST7789_DrawFilledTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 + 5, RGB565(255, 0, 0) );

  ST7789_print( x + w / 2 - 15, y + 155 - 16, RGB565(50, 150, 50) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, "----" );
}

// #########################################################################
//  функция опрашивает массив с данными и выводит их на экран
// #########################################################################
void plotPointer(void)
{
	int x = 0;	// устанавливаем смещение начального индикатора относительно х
	int y = 10; // устанавливаем смещение начального индикатора относительно у
	int count = 6; // устанавливаем кол-во индикаторов от 1 до 6
	
  int dy = 0;
  uint8_t pw = 16;

	// указываем кол-во индикаторов
  for (int i = 0; i < count; i++)
  {
	// формеруем и печатаем текущее значение в левом нижнем углу циферблата
	char buff_value[5];	// ставим на +1 чем нужно для \0
	sprintf( buff_value, "% 4d", value[i] );	// 4 знакоместа выравнивание по правому краю
	
	// выводим в низу индикатора данные с массивов
    ST7789_print( i * 40 + x + 36/2 -15, y + 27 - 27 + 155 - 16, RGB565(0, 0, 0) , RGB565(255, 255, 255) , 1, &Font_7x9, 1, buff_value );

    int dx = 3 + 40 * i;
	  // проверка на границы данных
    if (value[i] < 0){
		value[i] = 0; // Limit value to emulate needle end stops
	}
    if (value[i] > 100){
		value[i] = 100;
	}

    while (!(value[i] == old_value[i])) {
      dy = 27 + y + 100 - old_value[i];
      if (old_value[i] > value[i])
      {
        ST7789_DrawLine(x + dx, dy - 5, x + dx + pw, dy, RGB565(255, 255, 255));
        old_value[i]--;
        ST7789_DrawLine(x + dx, dy + 6, x + dx + pw, dy + 1, RGB565(255, 0, 0));
      }
      else
      {
        ST7789_DrawLine(x + dx, dy + 5, x + dx + pw, dy, RGB565(255, 255, 255));
        old_value[i]++;
        ST7789_DrawLine(x + dx, dy - 6, x + dx + pw, dy - 1, RGB565(255, 0, 0));
      }
    }
  }
}

//----------------------------------------------------------------------------------

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
