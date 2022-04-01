/**
  ******************************************************************************
  * @file    fytpi_control.h
  * @author  Brosy
  * @brief   控制相关
  * @date    2021-08-09
  ******************************************************************************
  * @attention 包括绝对式、增量式pid控制器、变结构PI控制器和ADRC等。
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
  *2022-01-30  |v1.1        |Brosy       |修改代码风格  
  *2021-08-28  |v1.0        |Brosy       |首次发布
  ******************************************************************************
**/

#ifndef __FYTPI_CONTROL_H
#define __FYTPI_CONTROL_H
/* includes ------------------------------------------------------------------*/
#include <stdint.h>

/* typedef -------------------------------------------------------------------*/
typedef struct _PIDIncrement
{
    /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
    float kp; /*< 比例系数 */
    float ki; /*< 积分系数 */
    float kd; /*< 微分系数 */

    float err_now;  /*< 当前的误差 */
    float d_ctr_out; /*< 控制增量输出 */
    float ctr_out;  /*< 控制输出 */

    float inc_lim;  /*< 增量限幅 */
    
    /* PID算法内部变量，其值不能修改 */
    float err_old_1;
    float err_old_2;
} PID_INCREMENT_T; /*< 增量式PID */

typedef struct _PIDAbsolute
{
    float kp;
    float ki;
    float kd;
    float err_i_lim_up;   //积分上限
    float err_i_lim_down; //积分上限
    float err_p_lim; //比例上限
    float err_lim;
    float err_now;
    float err_old;
    float err_p;
    float err_i;
    float err_d;
    float ctr_out;
} PID_ABSOLUTE_T; /*< 绝对式PID */

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/* 增量式PID */
void pidIncrementInit(PID_INCREMENT_T *PID, float kp, float ki, float kd,
    float IncLim);
float pidIncrementUpdate(float Target, float Current, PID_INCREMENT_T *PID);

/* 绝对式PID */
void pidAbsoluteInit(PID_ABSOLUTE_T *PID, float kp, float ki, float kd,
    float Errlimit, float ErrPLim);
float pidAbsoluteUpdate(float Target, float Current, PID_ABSOLUTE_T *PID);

/* 变结构PI（绝对式） */
void piTunningAbsolute(PID_ABSOLUTE_T* PID, float p_min, float p_dynamic, float i_min, float i_dynamic,
    float kd);

#endif
