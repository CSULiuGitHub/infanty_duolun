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
#ifndef _MOTOR_H
#define _MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "fytpi_control.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Mcircle_t
{
    int32_t circle;
    int16_t angle;
}M_CIRCLE_T;

typedef struct _MotorData_t
{
    int16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t tep;
    int16_t tar_current;
    int16_t output;
}MOTOR_DATA_T;

typedef struct _M2006_t
{
    M_CIRCLE_T circle;
    MOTOR_DATA_T Rx;
    MOTOR_DATA_T LPF;
    int16_t output;
}M2006_T;

typedef struct _M3508_t
{
    int16_t tar_speed;
    int16_t tar_current;
    MOTOR_DATA_T Rx;
    MOTOR_DATA_T LPF;
    PID_INCREMENT_T PidIncSpeed;
    PID_INCREMENT_T PidIncCurrent;
    int16_t output;
}M3508_T;

typedef struct _M6020_t
{
    int32_t continue_angle;
    M_CIRCLE_T Mc;
    MOTOR_DATA_T Rx;
    int32_t tar_speed;
    int16_t tar_current;
	  int32_t tar_angle;
	  int32_t angle;
    MOTOR_DATA_T LPF;
    PID_ABSOLUTE_T PidAbsAngle;
    PID_ABSOLUTE_T PidAbsSpeed;
    int32_t output;
}M6020_T;

typedef struct _GM6020_t
{
    int32_t continue_angle;
    M_CIRCLE_T Mc;
    MOTOR_DATA_T Rx;
}GM6020_T;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void circleContinue(M_CIRCLE_T *Mc, uint16_t angle);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
