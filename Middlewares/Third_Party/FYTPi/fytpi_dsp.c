/**
  ******************************************************************************
  * @file    fytpi_dsp.c
  * @author  Brosy
  * @brief   DSP
  * @date    2021-8-9
  ******************************************************************************
  * @attention 数字信号处理
  * 文件头注释，宽度为80个字符，为后续代码行宽提供参照。
  * 参考doxygen注释规范
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
  *<时间>      |<版本>      |<作者>      |<描述>      |  
  *2022-01-30  |v1.1        |Brosy       |修改代码风格|
  *2021-08-28  |v1.0        |Brosy       |首次发布    |
  ******************************************************************************
**/
  
/* includes ------------------------------------------------------------------*/
#include "fytpi_dsp.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/**
  * @brief  递推（滑动）平均滤波 4点
  * @param  原始数据，数据缓冲区
  * @retval 滤波后的值
  * @attention 
  */
int rm4Int(int raw_data, int rm4_filter_buf[4])
{
    float sum = 0;
    unsigned char i;
    
    for(i = 0; i < 3; i++) 
    {
		rm4_filter_buf[i] = rm4_filter_buf[i+1];  /*数组中所有数据左移一位 第一个数据扔掉*/
		sum += rm4_filter_buf[i];                 /*计算递推数组中处最后一个前面所有数据的和*/
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
		rm4_filter_buf[i] = rm4_filter_buf[i+1];  /*数组中所有数据左移一位 第一个数据扔掉*/
		sum += rm4_filter_buf[i];                 /*计算递推数组中处最后一个前面所有数据的和*/
	}
    
    rm4_filter_buf[3] = raw_data;
    
    sum =  rm4_filter_buf[0]*0.1f + rm4_filter_buf[1]*0.2f +rm4_filter_buf[2]*0.3f + rm4_filter_buf[3]*0.4f;
    return sum;
}
