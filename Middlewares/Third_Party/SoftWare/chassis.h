/**
  ******************************************************************************
  * @file    chassis.h
  * @author  施阳 胡小璐 宋其津 曾俊杰 周晓佳
  * @brief   底盘控制
  * @date    2022-01-30
  ******************************************************************************
  * @attention 无
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by fyt under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *修改记录：
  *<时间>      |<版本>      |<作者>      |<描述>     
  *2022-02-08  |v2.0        |qj          |首次发布
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CHASSIS_H
#define _CHASSIS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "motor.h"
#include "fytpi_control.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _MoveData
{
    int16_t right;   
    int16_t front;
    int32_t clock_wise;
}MOVE_DATA_T;

typedef struct _SuperCap
{
    uint16_t in_vol;
    uint16_t cap_vol;
    uint16_t in_cur;
    uint16_t power;
    uint16_t target_power;
    uint8_t  can_data[8];
}SUPER_CAP_T;

typedef struct _Chassis
{
    M3508_T m3508[4];
	  M6020_T m6020[4];
    uint8_t can_data[8];
	  uint8_t can_data1[8];
    SUPER_CAP_T SuperCap;
    MOVE_DATA_T MoveData;
}CHASSIS_T;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern CHASSIS_T Chassis;

/* function ------------------------------------------------------------------*/
void Chassis_Init(void);
void Chassis_Process(REMOTE_DATA_T RemoteMsg);
void Chassis_Protect(REMOTE_DATA_T RemoteMsg);
void Chassis_FastFixSpin(REMOTE_DATA_T RemoteMsg);
void Chassis_SpeedReset(void);
void Chassis_PowerLimit(void);
void Chassis_CanTransmit(void);/*< 放中断 */
int16_t Chassis_RosControl(float dir_angle);

void omniSpeedParse(float dir_angle, int16_t s16_speed_front,
                    int16_t s16_speed_right);
void keyboardControlGetMoveData(REMOTE_DATA_T RemoteMsg); /*< 键盘控制速度 */
void remoteControlGetMoveData(REMOTE_DATA_T RemoteMsg);   /*< 遥控器控制速度 */

int32_t dynamicSpin(void);
int16_t followHolderSpinS16(float dir_angle);
int16_t followSpeedSpinS16(float right_speed);
int16_t holderFollowGetMoveDataS16(REMOTE_DATA_T RemoteMsg);

#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
