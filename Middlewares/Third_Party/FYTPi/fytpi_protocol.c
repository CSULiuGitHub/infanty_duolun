/**
  ******************************************************************************
  * @file    fytpi_protocol.c
  * @author  Brosy
  * @brief   协议
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
#include "fytpi_protocol.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/**
  * @brief  遥控器接收机协议
  * @param  接收数据缓冲区，遥控数据结构体指针
  * @retval void
  * @attention
  */
void remoteDataMsgProcess(uint8_t *receive_data_buf, REMOTE_DATA_T *RemoteMsg)
{    
    RemoteMsg->ch0  = ((int16_t)receive_data_buf[0]       
                   | ((int16_t)receive_data_buf[1] << 8)) & 0x07FF;
    RemoteMsg->ch0 -= 1024;
    
    RemoteMsg->ch1  = (((int16_t)receive_data_buf[1] >> 3)
                   | ((int16_t)receive_data_buf[2] << 5)) & 0x07FF;
    RemoteMsg->ch1 -= 1024;
    
    RemoteMsg->ch2  = (((int16_t)receive_data_buf[2] >> 6)
                   | ((int16_t)receive_data_buf[3] << 2)  
                   | ((int16_t)receive_data_buf[4] << 10)) & 0x07FF;
    RemoteMsg->ch2 -= 1024;
    
    RemoteMsg->ch3  = (((int16_t)receive_data_buf[4] >> 1)
                   | ((int16_t)receive_data_buf[5] << 7)) & 0x07FF;
    RemoteMsg->ch3 -= 1024;
    
    RemoteMsg->s1   = (receive_data_buf[5] >> 6) & 0x03;
    RemoteMsg->s2   = (receive_data_buf[5] >> 4) & 0x03;
         
    RemoteMsg->mouse_speed_x = ((int16_t)receive_data_buf[6]
                          | (int16_t)receive_data_buf[7] << 8);
    RemoteMsg->mouse_speed_y = ((int16_t)receive_data_buf[8]
                          | (int16_t)receive_data_buf[9] << 8);
       
    RemoteMsg->mouse_click_left  = receive_data_buf[12];
    RemoteMsg->mouse_click_right = receive_data_buf[13]; 
      
    RemoteMsg->key     = ((int16_t)receive_data_buf[14]
                      | (int16_t)receive_data_buf[15] << 8);
    RemoteMsg->wheel   = ((int16_t)receive_data_buf[16]
                      |((int16_t)receive_data_buf[17] << 8)) & 0x07FF;
    RemoteMsg->wheel   = -RemoteMsg->wheel + 1024;

    if (RemoteMsg->ch0 > 660 || RemoteMsg->ch0 < -660)
    {
        RemoteMsg->ch0 = 0;
    }

    if (RemoteMsg->ch1 > 660 || RemoteMsg->ch1 < -660)
    {
        RemoteMsg->ch1 = 0;
    }
        
    if (RemoteMsg->ch2 > 660 || RemoteMsg->ch2 < -660)
    {
        RemoteMsg->ch2 = 0;
    }
        
    if (RemoteMsg->ch3 > 660 || RemoteMsg->ch3 < -660)
    {
        RemoteMsg->ch3 = 0;
    }
        
    if (RemoteMsg->wheel > 660 || RemoteMsg->wheel < -660)
    {
        RemoteMsg->wheel = 0;
    }

    RemoteMsg->KeyBoard.w     = RemoteMsg->key & KEY_PRESSED_OFFSET_W;
    RemoteMsg->KeyBoard.s     = (RemoteMsg->key & KEY_PRESSED_OFFSET_S)>>1;
    RemoteMsg->KeyBoard.a     = (RemoteMsg->key & KEY_PRESSED_OFFSET_A)>>2;
    RemoteMsg->KeyBoard.d     = (RemoteMsg->key & KEY_PRESSED_OFFSET_D)>>3;
    RemoteMsg->KeyBoard.shift = (RemoteMsg->key & KEY_PRESSED_OFFSET_SHIFT)>>4;
    RemoteMsg->KeyBoard.ctrl  = (RemoteMsg->key & KEY_PRESSED_OFFSET_CTRL)>>5;
    RemoteMsg->KeyBoard.q     = (RemoteMsg->key & KEY_PRESSED_OFFSET_Q)>>6;
    RemoteMsg->KeyBoard.e     = (RemoteMsg->key & KEY_PRESSED_OFFSET_E)>>7;
    RemoteMsg->KeyBoard.r     = (RemoteMsg->key & KEY_PRESSED_OFFSET_R)>>8;
    RemoteMsg->KeyBoard.f     = (RemoteMsg->key & KEY_PRESSED_OFFSET_F)>>9;
    RemoteMsg->KeyBoard.g     = (RemoteMsg->key & KEY_PRESSED_OFFSET_G)>>10;
    RemoteMsg->KeyBoard.z     = (RemoteMsg->key & KEY_PRESSED_OFFSET_Z)>>11;
    RemoteMsg->KeyBoard.x     = (RemoteMsg->key & KEY_PRESSED_OFFSET_X)>>12;
    RemoteMsg->KeyBoard.c     = (RemoteMsg->key & KEY_PRESSED_OFFSET_C)>>13;
    RemoteMsg->KeyBoard.v     = (RemoteMsg->key & KEY_PRESSED_OFFSET_V)>>14;
    RemoteMsg->KeyBoard.b     = (RemoteMsg->key & KEY_PRESSED_OFFSET_B)>>15; 
}
