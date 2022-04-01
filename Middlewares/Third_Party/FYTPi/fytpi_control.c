/**
  ******************************************************************************
  * @file    fytpi_control.c
  * @author  CSU_RM_FYT
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

/* includes ------------------------------------------------------------------*/
#include "fytpi_control.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/**
  * @brief  增量式pid初始化
  * @param  pid结构体地址，P,I,D,增量限幅
  * @retval void
  * @attention
  */
void pidIncrementInit(PID_INCREMENT_T *pid, float kp, float ki, float kd,
    float inc_lim)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->inc_lim = inc_lim;
}

/**
  * @brief  普通的增量式pid（可以进行增量限幅）
  * @param  目标值，反馈值，pid结构体地址
  * @retval 需要输出的量
  * @attention 返回的并非是增量，而直接是需要输出的值
  */
float pidIncrementUpdate(float target, float current, PID_INCREMENT_T *pid)
{
    float d_err_p, d_err_i, d_err_d;

    pid->err_now = target - current; /*< 误差量：目标-目前（相邻两次测量值取差分）*/

    d_err_p = pid->err_now - pid->err_old_1;
    d_err_i = pid->err_now;
    d_err_d = pid->err_now - 2 * pid->err_old_1 + pid->err_old_2;

    /* 增量式pid计算 */
    pid->d_ctr_out = pid->kp * d_err_p + pid->ki * d_err_i + pid->kd * d_err_d;

    pid->err_old_2 = pid->err_old_1; /*< 二阶误差微分 */
    pid->err_old_1 = pid->err_now;  /*< 一阶误差微分 */
    
    if (pid->d_ctr_out < -pid->inc_lim)
        pid->d_ctr_out = -pid->inc_lim;
    else if (pid->d_ctr_out > pid->inc_lim)
        pid->d_ctr_out = pid->inc_lim;

    pid->ctr_out += pid->d_ctr_out;

    return pid->ctr_out;
}

/**
  * @brief  绝对式pid初始化
  * @param  pid结构体地址，P,I,D,积分限幅,比例限幅
  * @retval void
  * @attention
  */
void pidAbsoluteInit(PID_ABSOLUTE_T *pid, float kp, float ki, float kd,
    float err_lim, float err_p_lim)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->err_lim = err_lim;
    pid->err_p_lim = err_p_lim;
}

/**
  * @brief  普通绝对式pid
  * @param  目标值，实际值，pid结构体地址
  * @retval 需要输出的值
  * @attention
  */
float pidAbsoluteUpdate(float target, float current, PID_ABSOLUTE_T *pid)
{
    pid->err_now = target - current;
    pid->err_p = pid->err_now;  /*< 读取现在的误差，用于kp控制 */
    pid->err_i += pid->err_now; /*< 误差积分，用于ki控制 */
    if (pid->err_lim != 0)     /*< 微分上限和下限 */
    {
        if (pid->err_i > pid->err_lim)
        {
            pid->err_i = pid->err_lim;
        }
        else if (pid->err_i < -pid->err_lim)
        {
            pid->err_i = -pid->err_lim;
        } 
    }
    
    /* 比例误差上限和下限 */
    if (pid->err_p_lim != 0)
    {
        if (pid->err_p > pid->err_p_lim)
        {
            pid->err_p = pid->err_p_lim;
        }
        else if (pid->err_p < -pid->err_p_lim)
        {
            pid->err_p = -pid->err_p_lim;
        }
    }

    pid->err_d = pid->err_now - pid->err_old; /*< 误差微分，用于kd控制 */
    pid->err_old = pid->err_now; /*< 保存现在的误差 */
    pid->ctr_out = pid->kp * pid->err_p + pid->ki * pid->err_i + pid->kd * pid->err_d; /*< 计算绝对式pid输出 */
    return pid->ctr_out;
}

/**
  * @brief  变结构PI控制器（绝对式）
  * @param  p最小值，p变化范围，i最小值，i变化范围，d值，绝对式pid结构体
  * @retval void
  * @attention
  */
void piTunningAbsolute(PID_ABSOLUTE_T* pid, float p_min, float p_dynamic, float i_min, float i_dynamic,
    float kd)
{
	float err = pid->err_now;
	
	err = err<0? -err : err; /*< 取绝对值*/
	
	pid->kp = p_min + p_dynamic*(1 - 1/(1+0.001f*(err+err*err))); /*< 误差大，P大，反应快 */
	
	pid->ki = i_min + i_dynamic*(1/(1+0.001f*(err+err*err))); /*< 误差小，I大， 稳定性强 */
	
    if (err < 20)
    {
        pid->ki = 0;
    }
    pid->kd = kd;
}
