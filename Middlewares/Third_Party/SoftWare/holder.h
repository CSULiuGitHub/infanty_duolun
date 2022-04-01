/**
  ******************************************************************************
  * @file    holder.h
  * @author  sy xl qj
  * @brief   云台任务
  * @date      
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HOLDER_H
#define _HOLDER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"

/* typedef -------------------------------------------------------------------*/
typedef struct _Yaw_t
{
    GM6020_T _0x20A;    
    PID_ABSOLUTE_T PidAbsSpeed;
    PID_ABSOLUTE_T PidAbsAngle;
	PID_ABSOLUTE_T PidAbsVision; 
    PID_INCREMENT_T PidIncSpeed;
    PID_INCREMENT_T PidIncAngle;
    int32_t tar_angle;
    int32_t tar_speed;
    int32_t tar_speed_lpf;
    int32_t angle;
    int32_t speed;
    int32_t output;
    int32_t output_lpf;
}YAW_T;

typedef struct _Pitch_t
{
    GM6020_T _0x209;
    PID_ABSOLUTE_T PidAbsSpeed;
    PID_ABSOLUTE_T PidAbsAngle;
  	PID_ABSOLUTE_T PidAbsVision;
    PID_INCREMENT_T PidIncSpeed;
    PID_INCREMENT_T PidIncAngle;
    int32_t tar_angle;
    int32_t tar_speed;
    int32_t tar_speed_lpf;
    int32_t angle;
    int32_t imu_angle;
    int32_t speed;
    int32_t output;
    int32_t output_lpf;
}PITCH_T;

typedef struct _Holder_t
{
    YAW_T Yaw;
    PITCH_T Pitch;
    uint8_t can_data[8];
}HOLDER_T;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void Holder_Init(HOLDER_DATA_T HolderMsg);
void Holder_Process(HOLDER_DATA_T HolderMsg,REMOTE_DATA_T RemoteMsg);

/*********************************云台辅助函数*********************************/
void Holder_Reset(void);
void Holder_Protect(REMOTE_DATA_T RemoteMsg);
void Holder_CanTransmit(void);
void Holder_Debug(void); /*< 云台调试 波形*/

void Holder_VisionPrediction(void);

extern HOLDER_T Holder;
//void Send_Vision(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
