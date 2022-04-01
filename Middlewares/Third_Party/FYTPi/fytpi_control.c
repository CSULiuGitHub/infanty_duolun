/**
  ******************************************************************************
  * @file    fytpi_control.c
  * @author  CSU_RM_FYT
  * @brief   �������
  * @date    2021-08-09
  ******************************************************************************
  * @attention ��������ʽ������ʽpid����������ṹPI��������ADRC�ȡ�
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by Brosy under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *�޸ļ�¼��
  *<ʱ��>      |<�汾>      |<����>      |<����>  
  *2022-01-30  |v1.1        |Brosy       |�޸Ĵ�����  
  *2021-08-28  |v1.0        |Brosy       |�״η���
  ******************************************************************************
**/

/* includes ------------------------------------------------------------------*/
#include "fytpi_control.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/**
  * @brief  ����ʽpid��ʼ��
  * @param  pid�ṹ���ַ��P,I,D,�����޷�
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
  * @brief  ��ͨ������ʽpid�����Խ��������޷���
  * @param  Ŀ��ֵ������ֵ��pid�ṹ���ַ
  * @retval ��Ҫ�������
  * @attention ���صĲ�������������ֱ������Ҫ�����ֵ
  */
float pidIncrementUpdate(float target, float current, PID_INCREMENT_T *pid)
{
    float d_err_p, d_err_i, d_err_d;

    pid->err_now = target - current; /*< �������Ŀ��-Ŀǰ���������β���ֵȡ��֣�*/

    d_err_p = pid->err_now - pid->err_old_1;
    d_err_i = pid->err_now;
    d_err_d = pid->err_now - 2 * pid->err_old_1 + pid->err_old_2;

    /* ����ʽpid���� */
    pid->d_ctr_out = pid->kp * d_err_p + pid->ki * d_err_i + pid->kd * d_err_d;

    pid->err_old_2 = pid->err_old_1; /*< �������΢�� */
    pid->err_old_1 = pid->err_now;  /*< һ�����΢�� */
    
    if (pid->d_ctr_out < -pid->inc_lim)
        pid->d_ctr_out = -pid->inc_lim;
    else if (pid->d_ctr_out > pid->inc_lim)
        pid->d_ctr_out = pid->inc_lim;

    pid->ctr_out += pid->d_ctr_out;

    return pid->ctr_out;
}

/**
  * @brief  ����ʽpid��ʼ��
  * @param  pid�ṹ���ַ��P,I,D,�����޷�,�����޷�
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
  * @brief  ��ͨ����ʽpid
  * @param  Ŀ��ֵ��ʵ��ֵ��pid�ṹ���ַ
  * @retval ��Ҫ�����ֵ
  * @attention
  */
float pidAbsoluteUpdate(float target, float current, PID_ABSOLUTE_T *pid)
{
    pid->err_now = target - current;
    pid->err_p = pid->err_now;  /*< ��ȡ���ڵ�������kp���� */
    pid->err_i += pid->err_now; /*< �����֣�����ki���� */
    if (pid->err_lim != 0)     /*< ΢�����޺����� */
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
    
    /* ����������޺����� */
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

    pid->err_d = pid->err_now - pid->err_old; /*< ���΢�֣�����kd���� */
    pid->err_old = pid->err_now; /*< �������ڵ���� */
    pid->ctr_out = pid->kp * pid->err_p + pid->ki * pid->err_i + pid->kd * pid->err_d; /*< �������ʽpid��� */
    return pid->ctr_out;
}

/**
  * @brief  ��ṹPI������������ʽ��
  * @param  p��Сֵ��p�仯��Χ��i��Сֵ��i�仯��Χ��dֵ������ʽpid�ṹ��
  * @retval void
  * @attention
  */
void piTunningAbsolute(PID_ABSOLUTE_T* pid, float p_min, float p_dynamic, float i_min, float i_dynamic,
    float kd)
{
	float err = pid->err_now;
	
	err = err<0? -err : err; /*< ȡ����ֵ*/
	
	pid->kp = p_min + p_dynamic*(1 - 1/(1+0.001f*(err+err*err))); /*< ����P�󣬷�Ӧ�� */
	
	pid->ki = i_min + i_dynamic*(1/(1+0.001f*(err+err*err))); /*< ���С��I�� �ȶ���ǿ */
	
    if (err < 20)
    {
        pid->ki = 0;
    }
    pid->kd = kd;
}
