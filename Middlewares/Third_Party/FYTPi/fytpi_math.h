/**
  ******************************************************************************
  * @file    fytpi_math.c
  * @author  Brosy
  * @brief   数学公式
  * @date    2021-08-09
  ******************************************************************************
  * @attention 包括三角函数、绝对值、平方根、限幅、斜坡等基础函数
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
  
#ifndef __FYTPI_MATH_H
#define __FYTPI_MATH_H

/* includes ------------------------------------------------------------------*/
#include <stdint.h>

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/* 三角函数 */
float fastCos(int angle);
float fastSin(int angle);

/* 限幅 */
int constrainInt(int num, int low, int high);
int8_t constrainInt8(int8_t num, int8_t low, int8_t high);
int16_t constrainInt16(int16_t num, int16_t low, int16_t high);
int32_t constrainInt32(int32_t num, int32_t low, int32_t high);
uint8_t constrainUint8(uint8_t num, uint8_t low, uint8_t high);
uint16_t constrainUint16(uint16_t num, uint16_t low, uint16_t high);
uint32_t constrainUint32(uint32_t num, uint32_t low, uint32_t high);
float constrainFloat(float num, float low, float high);

/* 斜坡 */
float rampFloat(float Final, float now, float ramp);
int rampInt(int Final, int now, int ramp);
int8_t rampInt8(int8_t Final, int8_t now, int8_t ramp);
int16_t rampInt16(int16_t Final, int16_t now, int16_t ramp);
int32_t rampInt32(int32_t Final, int32_t now, int32_t ramp);
uint8_t rampUint8(uint8_t Final, uint8_t now, uint8_t ramp);
uint16_t rampUint16(uint16_t Final, uint16_t now, uint16_t ramp);
uint32_t rampUint32(uint32_t Final, uint32_t now, uint32_t ramp);

/* 平方根 */
float quakeSqrt(float f_nuw);

/* 绝对值 */
float absFloat(float num);
int absInt(int num);
int8_t absInt8(int8_t num);
int16_t absInt16(int16_t num);
int32_t absInt32(int32_t num);

#endif
