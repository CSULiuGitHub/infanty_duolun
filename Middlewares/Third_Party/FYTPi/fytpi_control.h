/**
  ******************************************************************************
  * @file    fytpi_control.h
  * @author  Brosy
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

#ifndef __FYTPI_CONTROL_H
#define __FYTPI_CONTROL_H
/* includes ------------------------------------------------------------------*/
#include <stdint.h>

/* typedef -------------------------------------------------------------------*/
typedef struct _PIDIncrement
{
    /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
    float kp; /*< ����ϵ�� */
    float ki; /*< ����ϵ�� */
    float kd; /*< ΢��ϵ�� */

    float err_now;  /*< ��ǰ����� */
    float d_ctr_out; /*< ����������� */
    float ctr_out;  /*< ������� */

    float inc_lim;  /*< �����޷� */
    
    /* PID�㷨�ڲ���������ֵ�����޸� */
    float err_old_1;
    float err_old_2;
} PID_INCREMENT_T; /*< ����ʽPID */

typedef struct _PIDAbsolute
{
    float kp;
    float ki;
    float kd;
    float err_i_lim_up;   //��������
    float err_i_lim_down; //��������
    float err_p_lim; //��������
    float err_lim;
    float err_now;
    float err_old;
    float err_p;
    float err_i;
    float err_d;
    float ctr_out;
} PID_ABSOLUTE_T; /*< ����ʽPID */

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/* ����ʽPID */
void pidIncrementInit(PID_INCREMENT_T *PID, float kp, float ki, float kd,
    float IncLim);
float pidIncrementUpdate(float Target, float Current, PID_INCREMENT_T *PID);

/* ����ʽPID */
void pidAbsoluteInit(PID_ABSOLUTE_T *PID, float kp, float ki, float kd,
    float Errlimit, float ErrPLim);
float pidAbsoluteUpdate(float Target, float Current, PID_ABSOLUTE_T *PID);

/* ��ṹPI������ʽ�� */
void piTunningAbsolute(PID_ABSOLUTE_T* PID, float p_min, float p_dynamic, float i_min, float i_dynamic,
    float kd);

#endif
