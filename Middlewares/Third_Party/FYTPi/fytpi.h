/**
  ******************************************************************************
  * @file    fytpi.h
  * @author  Brosy
  * @brief   常用软件库
  * @date    2021-8-28
  ******************************************************************************
  * @attention 包含与嵌入式相关的数学公式、信号处理和自动控制等常见算法
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
  *2021-08-28  |v1.0        |Brosy       |首次发布    |
  ******************************************************************************
**/

#ifndef __FYTPI_H
#define __FYTPI_H

/* define --------------------------------------------------------------------*/
#define USE_FYTPI_DSP      (1) /*< 使用DSP库 */
#define USE_FYTPI_CONTROL  (1) /*< 使用Control库 */
#define USE_FYTPI_MATH     (0) /*< 不使用Math库 */
#define USE_FYTPI_PROTOCOL (0) /*< 不使用protocol库 */

/* includes ------------------------------------------------------------------*/
#if (USE_FYTPI_DSP)
    #include "fytpi_dsp.h"
#endif

#if (USE_FYTPI_CONTROL)
    #include "fytpi_control.h"
#endif

#if (USE_FYTPI_MATH)
    #include "fytpi_math.h"
#endif

#if (USE_FYTPI_PROTOCOL)
    #include "fytpi_protocol.h"
#endif

/* typedef -------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/

#endif
