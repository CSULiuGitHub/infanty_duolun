/**
  ******************************************************************************
  * @file    fytpi_protocol.h
  * @author  Brosy
  * @brief   通信协议
  * @date    2021-08-28
  ******************************************************************************
  * @attention 包括minipc、陀螺仪和遥控器串口协议，电机CAN协议
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by Brosy under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *修改记录：
  *<时间>      |<版本>      |<作者>      |<描述>     
  *2021-08-28  |v1.0        |Brosy       |首次发布
  ******************************************************************************
**/
  
/* includes ------------------------------------------------------------------*/
#include <stdint.h>

/* typedef -------------------------------------------------------------------*/
typedef struct _Kb16
{
	uint16_t w;
	uint16_t s;
	uint16_t a;
	uint16_t d;	
    uint16_t shift;
	uint16_t ctrl;
	uint16_t q;
	uint16_t e;
	uint16_t r;
	uint16_t f;
	uint16_t g;
	uint16_t z;
	uint16_t x;
	uint16_t c;
	uint16_t v;
	uint16_t b;
}KB16_T; /*< 键盘 */

typedef	struct _RemoteData
{
	int16_t	    ch0;	
	int16_t	    ch1;	
	int16_t	    ch2;	
	int16_t	    ch3;
	uint8_t	    s1;
	uint8_t	    s2;	
		
    int16_t	 mouse_speed_x;			
	int16_t	 mouse_speed_y;			
	uint8_t	 mouse_click_left;		
	uint8_t	 mouse_click_right;
    
    uint16_t    key;
    KB16_T      KeyBoard;
    
    int16_t     wheel;
}REMOTE_DATA_T; /*< 遥控数据 */

/* define --------------------------------------------------------------------*/
#define KEY_PRESSED_OFFSET_W	    (1U << 0)
#define KEY_PRESSED_OFFSET_S	    (1U << 1)
#define KEY_PRESSED_OFFSET_A	    (1U << 2)
#define KEY_PRESSED_OFFSET_D	    (1U << 3)
#define KEY_PRESSED_OFFSET_SHIFT	(1U << 4)
#define KEY_PRESSED_OFFSET_CTRL	    (1U << 5)
#define KEY_PRESSED_OFFSET_Q	    (1U << 6)
#define KEY_PRESSED_OFFSET_E	    (1U << 7)
#define KEY_PRESSED_OFFSET_R	    (1U << 8)
#define KEY_PRESSED_OFFSET_F	    (1U << 9)
#define KEY_PRESSED_OFFSET_G	    (1U << 10)
#define KEY_PRESSED_OFFSET_Z	    (1U << 11)
#define KEY_PRESSED_OFFSET_X	    (1U << 12)
#define KEY_PRESSED_OFFSET_C	    (1U << 13)
#define KEY_PRESSED_OFFSET_V	    (1U << 14)
#define KEY_PRESSED_OFFSET_B	    (1U << 15)

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
void remoteDataMsgProcess(uint8_t *receive_data_buf, REMOTE_DATA_T *RemoteMsg);
