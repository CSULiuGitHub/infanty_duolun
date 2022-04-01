/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "imu_data_decode.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
static M_CIRCLE_T mc_imu_yaw = {0};
static M_CIRCLE_T mc_imu_pitch = {0};
float Eular[3] = {0};
static int16_t gyro[3] = {0};
/* function ------------------------------------------------------------------*/
void RemoteDataMsg_Process(REMOTE_DATA_T *RemoteMsg)
{    
    RemoteMsg->Ch0  = (  (int16_t) uart1_buf[0]       | ( (int16_t) uart1_buf[1]  << 8 )) & 0x07FF;
    RemoteMsg->Ch0 -= 1024;
    RemoteMsg->Ch1  = (( (int16_t) uart1_buf[1] >> 3) | ( (int16_t) uart1_buf[2]  << 5 )) & 0x07FF;
    RemoteMsg->Ch1 -= 1024;
    RemoteMsg->Ch2  = (( (int16_t) uart1_buf[2] >> 6) | ( (int16_t) uart1_buf[3]  << 2 )  
                                                  | ( (int16_t) uart1_buf[4]  << 10)) & 0x07FF;
    RemoteMsg->Ch2 -= 1024;
    RemoteMsg->Ch3  = (( (int16_t) uart1_buf[4] >> 1) | ( (int16_t) uart1_buf[5]  << 7 )) & 0x07FF;
    RemoteMsg->Ch3 -= 1024;
    RemoteMsg->S1   = (            uart1_buf[5] >> 6)                                     & 0x03;
    RemoteMsg->S2   = (            uart1_buf[5] >> 4)                                     & 0x03;
         
    RemoteMsg->Mouse_x = ( (int16_t) uart1_buf[6] | (int16_t) uart1_buf[7] << 8);
    RemoteMsg->Mouse_y = ( (int16_t) uart1_buf[8] | (int16_t) uart1_buf[9] << 8);
       
    RemoteMsg->MouseClick_left  = uart1_buf[12];
    RemoteMsg->MouseClick_right = uart1_buf[13]; 
      
    RemoteMsg->Key     = ( (int16_t) uart1_buf[14] |   (int16_t) uart1_buf[15] << 8 );
    RemoteMsg->Wheel   = ( (int16_t) uart1_buf[16] | ( (int16_t) uart1_buf[17] << 8 )) & 0x07FF;
    RemoteMsg->Wheel   = -RemoteMsg->Wheel + 1024;

    if(RemoteMsg->Ch0 > 660 || RemoteMsg->Ch0 < -660)
        RemoteMsg->Ch0 = 0;
    if(RemoteMsg->Ch1 > 660 || RemoteMsg->Ch1 < -660)
        RemoteMsg->Ch1 = 0;
    if(RemoteMsg->Ch2 > 660 || RemoteMsg->Ch2 < -660)
        RemoteMsg->Ch2 = 0;
    if(RemoteMsg->Ch3 > 660 || RemoteMsg->Ch3 < -660)
        RemoteMsg->Ch3 = 0;
    if(RemoteMsg->Wheel > 660 || RemoteMsg->Wheel < -660)
        RemoteMsg->Wheel = 0;    
    
    RemoteMsg->KeyBoard.w     = RemoteMsg->Key & KEY_PRESSED_OFFSET_W;
    RemoteMsg->KeyBoard.s     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_S)>>1;
    RemoteMsg->KeyBoard.a     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_A)>>2;
    RemoteMsg->KeyBoard.d     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_D)>>3;
    RemoteMsg->KeyBoard.shift = (RemoteMsg->Key & KEY_PRESSED_OFFSET_SHIFT)>>4;
    RemoteMsg->KeyBoard.ctrl  = (RemoteMsg->Key & KEY_PRESSED_OFFSET_CTRL)>>5;
    RemoteMsg->KeyBoard.q     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_Q)>>6;
    RemoteMsg->KeyBoard.e     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_E)>>7;
    RemoteMsg->KeyBoard.r     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_R)>>8;
    RemoteMsg->KeyBoard.f     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_F)>>9;
    RemoteMsg->KeyBoard.g     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_G)>>10;
    RemoteMsg->KeyBoard.z     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_Z)>>11;
    RemoteMsg->KeyBoard.x     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_X)>>12;
    RemoteMsg->KeyBoard.c     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_C)>>13;
    RemoteMsg->KeyBoard.v     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_V)>>14;
    RemoteMsg->KeyBoard.b     = (RemoteMsg->Key & KEY_PRESSED_OFFSET_B)>>15; 
}

static float IMUAngle_Continue(M_CIRCLE_T *McImu,float imu_angle)
{
	float out_data = 0; 

	circleContinue(McImu, (imu_angle + 180.0f) * 22.7555556f);
	out_data = 819 * (float)(McImu->circle + (float)(McImu->angle) / 8192);

	return out_data;    
}
float look6;
void HolderDataMsg_Process(HOLDER_DATA_T *HolderMsg)
{
    static float angle2speed[3];
    get_eular(Eular);
    
    HolderMsg->angle[2] = IMUAngle_Continue(&mc_imu_yaw,Eular[2]);
    angle2speed[2] = HolderMsg->angle[2] - angle2speed[2];
    HolderMsg->speed[2] = angle2speed[2];
    angle2speed[2] = HolderMsg->angle[2];
    look6 = HolderMsg->angle[2];
    HolderMsg->angle[0] = IMUAngle_Continue(&mc_imu_pitch,Eular[0]);
    
    get_raw_gyo(gyro);
    HolderMsg->gyro[2] = -(gyro[2]);
    HolderMsg->gyro[0] = -gyro[0];
}


/*********************************ÏûÏ¢¸¨Öúº¯Êý*********************************/

M_CIRCLE_T* Get_Mc_Imu_Pitch(void)
{
    return &mc_imu_pitch;
}

float getImuYaw(void)
{
    return Eular[2];
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
