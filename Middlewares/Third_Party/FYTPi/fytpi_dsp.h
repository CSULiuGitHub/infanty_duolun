/**
  ******************************************************************************
  * @file    fytpi_dsp.h
  * @author  Brosy
  * @brief   DSP
  * @date    2021-8-9
  ******************************************************************************
  * @attention 四点加权滤波
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

#ifndef __FYTPI_MATH_H
#define __FYTPI_MATH_H

/* includes ------------------------------------------------------------------*/
#include <stdint.h>

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
int rm4Int(int raw_data, int rm4_filter_buf[4]);
float rm4Float(float raw_data, float rm4_filter_buf[4]);
    
#endif
