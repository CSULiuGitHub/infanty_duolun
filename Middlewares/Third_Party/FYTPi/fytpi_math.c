/**
  ******************************************************************************
  * @file    fytpi_math.c
  * @author  Brosy
  * @brief   常用数学函数
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
  
/* includes ------------------------------------------------------------------*/
#include "fytpi_math.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
static float fast_cos_buf_[91] = { 1,
    0.999848,0.999391,0.99863,0.997564,0.996195,0.994522,0.992546,0.990268,
    0.987688,0.984808,0.981627,0.978148,0.97437,0.970296,0.965926,0.961262,
    0.956305,0.951057,0.945519,0.939693,0.93358,0.927184,0.920505,0.913545,
    0.906308,0.898794,0.891007,0.882948,0.87462,0.866025,0.857167,0.848048,
    0.838671,0.829038,0.819152,0.809017,0.798635,0.788011,0.777146,0.766044,
    0.75471,0.743145,0.731354,0.71934,0.707107,0.694658,0.681998,0.669131,
    0.656059,0.642788,0.62932,0.615661,0.601815,0.587785,0.573576,0.559193,
    0.544639,0.529919,0.515038,0.5,0.48481,0.469471,0.45399,0.438371,
    0.422618,0.406737,0.390731,0.374606,0.358368,0.34202,0.325568,0.309017,
    0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,
    0.156434,0.139173,0.121869,0.104528,0.0871556,0.0697563,0.0523358,
    0.0348993,0.0174522,-1.73205e-07};

/* function ------------------------------------------------------------------*/
/**
  * @brief  快速计算cos
  * @param  整数角度值，单位“度”
  * @retval 计算值
  * @attention 根据三角函数周期性和对称性，只需取90个已知值就能计算出所有角度值
  */
float fastCos(int angle)
{
	if (angle>=0 && angle <= 90)
	{
		return fast_cos_buf_[angle];
	}
	else if (angle > 90 && angle <=180)
	{
		return -(fast_cos_buf_[180-angle]);
	}
	else if (angle > 180 && angle <=360)
	{
		return fastCos(360-angle);
	}
	else if (angle > 360)
	{
		return fastCos(angle - 360);
	}
	else if (angle < 0)
	{
		return (fastCos(-angle));
	}
	return 0;
}

/**
  * @brief  快速计算sin
  * @param  整数角度值，单位“度”
  * @retval 计算值
  * @attention 根据三角函数周期性和对称性，只需取90个已知值就能计算出所有角度值
  */
float fastSin(int angle)
{
	return fastCos(angle - 90.0f);
}

/**
  * @brief  限幅
  * @param  目标变量，最小值，最大值
  * @retval 对应类型限幅值
  * @attention 限幅函数比较常用，大都在任务中循环执行
  */
int constrainInt(int num, int low, int high)
{
    return num<low? low:(num>high? high:num);
}

int8_t constrainInt8(int8_t num, int8_t low, int8_t high)
{
    return num<low? low:(num>high? high:num);
}

int16_t constrainInt16(int16_t num, int16_t low, int16_t high)
{
    return num<low? low:(num>high? high:num);
}

int32_t constrainInt32(int32_t num, int32_t low, int32_t high)
{
    return num<low? low:(num>high? high:num);
}

uint8_t constrainUint8(uint8_t num, uint8_t low, uint8_t high)
{
    return num<low? low:(num>high? high:num);
}

uint16_t constrainUint16(uint16_t num, uint16_t low, uint16_t high)
{
    return num<low? low:(num>high? high:num);
}

uint32_t constrainUint32(uint32_t num, uint32_t low, uint32_t high)
{
    return num<low? low:(num>high? high:num);
}

float constrainFloat(float num, float low, float high)
{
	return num<low? low:(num>high? high:num);
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(每次增加的值)
  * @retval 当前输出
  * @attention  
  */
float rampFloat(float final, float now, float ramp)
{
    float buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

int rampInt(int final, int now, int ramp)
{
    int buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

int8_t rampInt8(int8_t final, int8_t now, int8_t ramp)
{
    int buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

int16_t rampInt16(int16_t final, int16_t now, int16_t ramp)
{
    int buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

int32_t rampInt32(int32_t final, int32_t now, int32_t ramp)
{
    int buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

uint8_t rampUint8(uint8_t final, uint8_t now, uint8_t ramp)
{
    uint8_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

uint16_t rampUint16(uint16_t final, uint16_t now, uint16_t ramp)
{
    uint16_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

uint32_t rampUint32(uint32_t final, uint32_t now, uint32_t ramp)
{
    uint32_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  Quake-III Arena
  * @param  x
  * @retval 根号x
  * @attention  大神写的快速平方根    
  */
float quakeSqrt(float f_num)
{
    if(f_num == 0) return 0; 
    float result = f_num; 
    float num_half = 0.5f*result; 
    int i = *(int*)&result; 

    i = 0x5f3759df - (i>>1); 
    result = *(float*)&i; 
    result = result*(1.5f-num_half*result*result);
    result = result*(1.5f-num_half*result*result); 
    return 1.0f/result; 
}

/**
  * @brief  float取绝对值
  * @param  num
  * @retval |num|
  * @attention  
  */
float absFloat(float num)
{
    float abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

/**
  * @brief  float取绝对值
  * @param  num
  * @retval |num|
  * @attention  
  */
int absInt(int num)
{
    int abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

int8_t absInt8(int8_t num)
{
    int8_t abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

int16_t absInt16(int16_t num)
{
    int16_t abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

int32_t absInt32(int32_t num)
{
    int32_t abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

/* end -----------------------------------------------------------------------*/
