/**
  ******************************************************************************
  * @file    fytpi.h
  * @author  Brosy
  * @brief   ���������
  * @date    2021-8-28
  ******************************************************************************
  * @attention ������Ƕ��ʽ��ص���ѧ��ʽ���źŴ�����Զ����Ƶȳ����㷨
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
  *2021-08-28  |v1.0        |Brosy       |�״η���    |
  ******************************************************************************
**/

#ifndef __FYTPI_H
#define __FYTPI_H

/* define --------------------------------------------------------------------*/
#define USE_FYTPI_DSP      (1) /*< ʹ��DSP�� */
#define USE_FYTPI_CONTROL  (1) /*< ʹ��Control�� */
#define USE_FYTPI_MATH     (0) /*< ��ʹ��Math�� */
#define USE_FYTPI_PROTOCOL (0) /*< ��ʹ��protocol�� */

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
