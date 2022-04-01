/**
  ******************************************************************************
  * @file    fytpi_dsp.c
  * @author  Brosy
  * @brief   DSP
  * @date    2021-8-9
  ******************************************************************************
  * @attention �����źŴ���
  * �ļ�ͷע�ͣ����Ϊ80���ַ���Ϊ���������п��ṩ���ա�
  * �ο�doxygenע�͹淶
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
  *<ʱ��>      |<�汾>      |<����>      |<����>      |  
  *2022-01-30  |v1.1        |Brosy       |�޸Ĵ�����|
  *2021-08-28  |v1.0        |Brosy       |�״η���    |
  ******************************************************************************
**/
  
/* includes ------------------------------------------------------------------*/
#include "fytpi_dsp.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/**
  * @brief  ���ƣ�������ƽ���˲� 4��
  * @param  ԭʼ���ݣ����ݻ�����
  * @retval �˲����ֵ
  * @attention 
  */
int rm4Int(int raw_data, int rm4_filter_buf[4])
{
    float sum = 0;
    unsigned char i;
    
    for(i = 0; i < 3; i++) 
    {
		rm4_filter_buf[i] = rm4_filter_buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += rm4_filter_buf[i];                 /*������������д����һ��ǰ���������ݵĺ�*/
	}
    
    rm4_filter_buf[3] = raw_data;
    
    sum =  rm4_filter_buf[0]*0.1f + rm4_filter_buf[1]*0.2f +rm4_filter_buf[2]*0.3f + rm4_filter_buf[3]*0.4f;
    return sum;
}

float rm4Float(float raw_data, float rm4_filter_buf[4])
{
    float sum = 0;
    uint8_t i;
    for(i = 0; i < 3; i++) 
    {
		rm4_filter_buf[i] = rm4_filter_buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += rm4_filter_buf[i];                 /*������������д����һ��ǰ���������ݵĺ�*/
	}
    
    rm4_filter_buf[3] = raw_data;
    
    sum =  rm4_filter_buf[0]*0.1f + rm4_filter_buf[1]*0.2f +rm4_filter_buf[2]*0.3f + rm4_filter_buf[3]*0.4f;
    return sum;
}
