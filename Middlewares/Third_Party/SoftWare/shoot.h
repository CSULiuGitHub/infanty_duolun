/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief   
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
#ifndef _SHOOT_H
#define _SHOOT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
#include "fytpi_control.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Shoot_t
{
      M_CIRCLE_T Mcircle_Position;
      M2006_T InformarionTran;
	  int32_t speed;
      int32_t rmf_speed; /*< 递推均值滤波 */
      int32_t angle; 
	  int32_t tar_position;
      int32_t position;
	  int32_t tar_speed;
      int32_t tar_speed_lpf;
	  int32_t output;
      int32_t output_lpf;
      
	  uint8_t can_data[8];
      PID_ABSOLUTE_T PidAbsSpeed;
	  PID_ABSOLUTE_T PidAbsPosition;
}SHOOT_T;

typedef struct _Shoot_Speed_t
{
      M3508_T Can;
	  int32_t speed;
      int32_t rmf_speed; /*< 加权滤波 */
	  int32_t tar_speed;
      int32_t tar_speed_lpf;
	  int32_t output;
      int32_t output_lpf;
	  uint8_t can_data[8];
      PID_ABSOLUTE_T PidAbsSpeed;
}SHOOT_SPEED_T;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern SHOOT_T M2006_ID4;
extern SHOOT_T M2006_ID5;
extern SHOOT_SPEED_T M3508[2];

/* function ------------------------------------------------------------------*/
void Shoot_FreqPidReset(void); /*< 拨弹电机PID误差清零 */
void Shoot_PidInit(void);
void Shoot_Process(REMOTE_DATA_T RemoteMsg);
void Shoot_CanTransmit(void);
void Shoot_FreqControl(REMOTE_DATA_T RemoteMsg); /*< 连发速率控制*/
void Shoot_SpeedControlRealtime(void); /*< 实时射速控制*/
void Shoot_SpeedPidInitM3508(void);
void Shoot_SpeedCanTransmitM3508(void);
void bulletPrepare(REMOTE_DATA_T RemoteMsg);
void lidOpen(void);
void lidClose(void);

#ifdef __cplusplus
}
#endif

#endif /* define*/
