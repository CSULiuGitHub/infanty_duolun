/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
#include "RTE_Components.h"             // Component selection
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "motor.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
#define CIRCLE_FASTEST 1000
void circleContinue(M_CIRCLE_T *mc, uint16_t angle)
{
    if ((angle < CIRCLE_FASTEST) && (mc->angle > 8192 - CIRCLE_FASTEST))
    {
        mc->circle++;
    }
    else if ((angle > 8192 - CIRCLE_FASTEST) && (mc->angle < CIRCLE_FASTEST))
    {
        mc->circle--;
    }
    mc->angle = angle;
}



/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
