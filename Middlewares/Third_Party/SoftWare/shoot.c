/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
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
#include <string.h>
#include <math.h>
#include "config.h"
#include "shoot_config.h"
#include "shoot.h"
#include "motor.h"
#include "vision.h"
#include "tim.h"
#include "usart.h"
#include "can.h"
#include "judge.h"
#include "judge_tx.h"
#include "ui.h"
#include "misc_func.h"
#include "holder.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
portTickType posishoot_time;//射击延时测试
SHOOT_T M2006_ID4 = {0};
SHOOT_T M2006_ID5 = {0};
uint16_t heat17_limit;
int16_t s16shoot_freq = 500;
int16_t s16shoot_speed = 8000;
int32_t cmp_position = 0;
SHOOT_SPEED_T M3508[2]; 
REMOTE_DATA_T RDMsg1;

/* function ------------------------------------------------------------------*/
/**
  * @brief  拨盘PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void  Shoot_PidInit(void)
{   
    pidAbsoluteInit(&M2006_ID4.PidAbsPosition,Shoot_Freq_Position_Kp,Shoot_Freq_Position_Ki,Shoot_Freq_Position_Kd,
                      Shoot_Freq_Position_Ki_Limit,Shoot_Freq_Position_Kp_Limit);
    
    pidAbsoluteInit(&M2006_ID4.PidAbsSpeed,Shoot_Freq_Speed_Kp,Shoot_Freq_Speed_Ki,Shoot_Freq_Speed_Kd,
                      Shoot_Freq_Speed_Ki_Limit,Shoot_Freq_Speed_Kp_Limit);
	
	  pidAbsoluteInit(&M2006_ID5.PidAbsPosition,Shoot_Freq_Position_Kp,Shoot_Freq_Position_Ki,Shoot_Freq_Position_Kd,
                      Shoot_Freq_Position_Ki_Limit,Shoot_Freq_Position_Kp_Limit);
    
    pidAbsoluteInit(&M2006_ID5.PidAbsSpeed,Shoot_Freq_Speed_Kp,Shoot_Freq_Speed_Ki,Shoot_Freq_Speed_Kd,
                      Shoot_Freq_Speed_Ki_Limit,Shoot_Freq_Speed_Kp_Limit);

    #ifdef INFANTRY_6
    Shoot_SpeedPidInitM3508();
    #endif
}


/**
  * @brief  拨盘PID输出
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
int32_t ID4_RMF4_Speed_Buf[4];  /*< 速度反馈四点加权滤波缓冲区 */
int32_t ID5_RMF4_Speed_Buf[4];
int32_t RMF4_Output_Buf[4]; /*< 输出四点加权滤波缓冲区 */
void Shoot_PidRun(REMOTE_DATA_T RemoteMsg)
{
    #ifdef SHOOT_FREQ_DEBUG
        Shoot_PidInit();
    #endif
    
    M2006_ID4.speed = M2006_ID4.InformarionTran.Rx.speed;
    M2006_ID4.position = 1000*((float)M2006_ID4.Mcircle_Position.circle + (float)M2006_ID4.Mcircle_Position.angle/8192.0f);
    M2006_ID4.rmf_speed = Misc_s32Recursive_Mean4_Filter(M2006_ID4.speed,ID4_RMF4_Speed_Buf); /*< 四点加权滤波 */
    
    M2006_ID4.tar_speed = pidAbsoluteUpdate(M2006_ID4.tar_position, M2006_ID4.position, &M2006_ID4.PidAbsPosition);
    M2006_ID4.output_lpf = pidAbsoluteUpdate(M2006_ID4.tar_speed,M2006_ID4.speed,&M2006_ID4.PidAbsSpeed);
    M2006_ID4.output = M2006_ID4.output_lpf*0.7 + M2006_ID4.output*0.3;
    M2006_ID4.output = Constrain_Int32_t(M2006_ID4.output, -Shoot_Freq_Out_Limit, Shoot_Freq_Out_Limit);
    
    #ifdef INFANTRY_5_DOUBLE
	  M2006_ID5.position = 1000*((float)M2006_ID5.Mcircle_Position.circle + (float)M2006_ID5.Mcircle_Position.angle/8192.0f);
    M2006_ID5.speed = M2006_ID5.InformarionTran.Rx.speed;
    M2006_ID5.rmf_speed = Misc_s32Recursive_Mean4_Filter(M2006_ID5.speed,ID5_RMF4_Speed_Buf); /*< 四点加权滤波 */
    
    M2006_ID5.tar_speed = pidAbsoluteUpdate(M2006_ID5.tar_position, M2006_ID5.position, &M2006_ID5.PidAbsPosition);
    M2006_ID5.output_lpf = pidAbsoluteUpdate(M2006_ID5.tar_speed,M2006_ID5.rmf_speed,&M2006_ID5.PidAbsSpeed);
    M2006_ID5.output = M2006_ID5.output_lpf*0.7 + M2006_ID5.output*0.3;
    M2006_ID5.output = Constrain_Int32_t(M2006_ID5.output, -Shoot_Freq_Out_Limit, Shoot_Freq_Out_Limit);
    #endif
    
    #ifdef SHOOT_FREQ_WAVE
//        UART2_SendWave(5, 4, &M2006_ID4.tar_angle, &M2006_ID4.angle, 
//                            &M2006_ID4.tar_speed, &M2006_ID4.rmf_speed, &M2006_ID4.output);
    #endif
}


void  Shoot_SpeedPidInitM3508(void)
{
    #ifdef INFANTRY_6
    pidAbsoluteInit(&M3508[0].PidAbsSpeed,Shoot_Freq_Speed_Kp_M3508,Shoot_Freq_Speed_Ki_M3508,Shoot_Freq_Speed_Kd_M3508,
                      Shoot_Freq_Speed_Ki_Limit_M3508,Shoot_Freq_Speed_Kp_Limit_M3508);
    pidAbsoluteInit(&M3508[1].PidAbsSpeed,Shoot_Freq_Speed_Kp_M3508,Shoot_Freq_Speed_Ki_M3508,Shoot_Freq_Speed_Kd_M3508,
                      Shoot_Freq_Speed_Ki_Limit_M3508,Shoot_Freq_Speed_Kp_Limit_M3508 );
    #endif
}

int32_t RMF4_Speed_M3508_Buf[2][4];  /*< 速度反馈四点加权滤波缓冲区 */

int16_t m3508_speed_now[2];
void Shoot_Speed_Pid_Run_M3508(void)
{
    #ifdef SHOOT_M3508_DEBUG
        Shoot_SpeedPidInitM3508();
    #endif
    uint8_t i;
    
    /* 斜坡函数，缓慢提升速度，加速过大会导致云台抖动 */
    M3508[0].tar_speed = Misc_RAMP_Int16(-s16shoot_speed, m3508_speed_now[0], 4);
    M3508[1].tar_speed = Misc_RAMP_Int16(s16shoot_speed, m3508_speed_now[1], 4);
    m3508_speed_now[0] = M3508[0].tar_speed;
    m3508_speed_now[1] = M3508[1].tar_speed;
    
    for(i=0; i<2; i++)
    {
        M3508[i].rmf_speed = Misc_s32Recursive_Mean4_Filter(M3508[i].Can.Rx.speed, RMF4_Speed_M3508_Buf[i]);
        M3508[i].output_lpf = pidAbsoluteUpdate(M3508[i].tar_speed, M3508[i].rmf_speed, &M3508[i].PidAbsSpeed);
        M3508[i].output = M3508[i].output_lpf*0.7 + M3508[i].output*0.3;
        M3508[i].output = Constrain_Int32_t(M3508[i].output, -M3508_Out_Limit, M3508_Out_Limit);
    }

    #ifdef SHOOT_M3508_WAVE
//        UART2_SendWave(6, 4, &M3508[0].tar_speed, &M3508[0].rmf_speed, &M3508[0].output, 
//                       &M3508[1].tar_speed, &M3508[1].rmf_speed, &M3508[1].output);
    #endif
}


/**
  * @brief  拨弹状态机
  * @param  控制指令结构体
  * @retval 表示拨弹状态的16位数字
  * @attention 
  */
typedef enum
{
	start  = 0,     
	wait   = 1,
	review = 2
}launch;


/**
  * @brief  拨盘CAN发送
  * @param  void
  * @retval void
  * @attention 
  */
float shoot_aim_pulse = 0;
float shoot_now_pulse = 0;
float shoot_ramp_pulse = 0.05;
void Shoot_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)
    {
        M2006_ID4.can_data[6]=(uint8_t)(M2006_ID4.output>>8);
        M2006_ID4.can_data[7]=(uint8_t)(M2006_ID4.output);
		
        #ifdef INFANTRY_5_DOUBLE
		    M2006_ID5.can_data[0]=(uint8_t)(M2006_ID5.output>>8);
        M2006_ID5.can_data[1]=(uint8_t)(M2006_ID5.output);
        #endif
		
        shoot_aim_pulse = RAMP_float(Pulse_Out, shoot_now_pulse, shoot_ramp_pulse);
        shoot_now_pulse = shoot_aim_pulse;
        TIM1_SetPWMPluse(shoot_aim_pulse);
        TIM3_SetPWMPluse(shoot_aim_pulse);
        TIM9_SetPWMPluse(shoot_aim_pulse);
    }   
    else
    {
        Shoot_FreqPidReset();
        memset(M2006_ID4.can_data,0,sizeof(M2006_ID4.can_data));
        
        #ifdef INFANTRY_5_DOUBLE
        memset(M2006_ID5.can_data,0,sizeof(M2006_ID5.can_data));
        #endif
        
		TIM1_SetPWMPluse(200);
		TIM3_SetPWMPluse(200);
		TIM9_SetPWMPluse(200);
    }
    CAN1_Transmit(0x200,M2006_ID4.can_data);
    
    #ifdef INFANTRY_5_DOUBLE
    CAN1_Transmit(0x1FF,M2006_ID5.can_data);
    #endif
}

/**
  * @brief  摩擦轮CAN发送
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_SpeedCanTransmitM3508(void)
{
    uint8_t can_data[8];
    if(Observer.Tx.DR16_Rate>15)
    {
        can_data[0] = M3508[0].output>>8;
        can_data[1] = M3508[0].output;
        can_data[4] = M3508[1].output>>8;
        can_data[5] = M3508[1].output;
			  can_data[6] = M2006_ID4.output>>8;
        can_data[7] = M2006_ID4.output;
    }
    else
    {
        memset(can_data,0,sizeof(can_data));
    }
    CAN1_Transmit(0x200,can_data);
}

/**
  * @brief  选择拨弹形式
  * @param  表示拨弹状态的16位数字
  * @retval void
  * @attention 
  */

void Shoot_Mode_Choose(REMOTE_DATA_T RemoteMsg)
{
    uint8_t wheel;
    uint8_t shoot_mode;
	  RDMsg1=RemoteMsg;
    
    /* 热量控制 */
    uint16_t heat17 = JUDGE_u16GetRemoteHeat17();
	  heat17_limit = JUDGE_u16GetHeatLimit();
    
    wheel = RemoteMsg.Wheel> 100 ? 1:0;
	
    shoot_mode = RemoteMsg.MouseClick_left || wheel; /*< 按住置1， 松开置0*/
    
    if (heat17_limit < 40)
    {
        heat17_limit = 40;
    }
    
    if (heat17 >= heat17_limit-15)
    {
        shoot_mode = 0;
    }
    
    
    switch(shoot_mode)
    {
        case 0: /*< 停止 */
            
        break;

        case 1: /*< 发射 */
            
            #ifdef INFANTRY_5_DOUBLE
            if (M2006_ID5.tar_position - M2006_ID5.position < s16shoot_freq)
            {
                M2006_ID5.tar_position += 4000;
            }
			      if (M2006_ID4.tar_position - M2006_ID4.position > -s16shoot_freq)
            {
                M2006_ID4.tar_position -= 4000;
            }
            #else
            if (M2006_ID4.tar_position - M2006_ID4.position > -s16shoot_freq)
            {
                M2006_ID4.tar_position -= 2200;
            }
            #endif
        break;

        default:
            
        break;
    }
}
/**
  * @brief  拨盘进程
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Shoot_Process(REMOTE_DATA_T RemoteMsg)
{    
    Shoot_Mode_Choose(RemoteMsg);
    Shoot_FreqControl(RemoteMsg);
	  Shoot_PidRun(RemoteMsg);
	  bulletPrepare(RemoteMsg);
    Shoot_SpeedControlRealtime();
		Shoot_Speed_Pid_Run_M3508();
//    #ifdef INFANTRY_6
//    Shoot_Speed_Pid_Run_M3508();
//    #endif
    Shoot_CanTransmit();
}

/*********************************发射辅助函数*********************************/

void Shoot_FreqPidReset()
{
    cmp_position = M2006_ID4.position;
    M2006_ID4.tar_position = M2006_ID4.position;
    M2006_ID4.tar_speed = M2006_ID4.rmf_speed;
    M2006_ID4.PidAbsSpeed.err_old = 0;
    M2006_ID4.PidAbsSpeed.err_p = 0;
    M2006_ID4.PidAbsSpeed.err_now = 0;
    M2006_ID4.PidAbsSpeed.ctr_out = 0;
    M2006_ID4.PidAbsSpeed.err_d = 0;
    M2006_ID4.PidAbsSpeed.err_i = 0;
	
	cmp_position = M2006_ID5.position;
    M2006_ID5.tar_position = M2006_ID5.position;
    M2006_ID5.tar_speed = M2006_ID5.rmf_speed;
    M2006_ID5.PidAbsSpeed.err_old = 0;
    M2006_ID5.PidAbsSpeed.err_p = 0;
    M2006_ID5.PidAbsSpeed.err_now = 0;
    M2006_ID5.PidAbsSpeed.ctr_out = 0;
    M2006_ID5.PidAbsSpeed.err_d = 0;
    M2006_ID5.PidAbsSpeed.err_i = 0;
}

/**
  * @brief  实时射速控制
  * @param
  * @retval void
  * @attention  射速控制，尽量稳定在目标射速
  */

void Shoot_SpeedControlRealtime()
{
    uint16_t max_pulse = Pusel_Max_Lim15;
    
    uint16_t speed_limit = JUDGE_u16GetSpeedHeat17Limit(); /*< 裁判限速 */
    
    switch(speed_limit)
    {
        case 15:
            max_pulse = Pusel_Max_Lim15;
            s16shoot_speed = Shoot_Speed_Max_M3508_Lim15;
        break;
        
        case 18:
            max_pulse = Pusel_Max_Lim18;
            s16shoot_speed = Shoot_Speed_Max_M3508_Lim18;
        break;
        
        case 22:
            max_pulse = Pusel_Max_Lim22;
            s16shoot_speed = Shoot_Speed_Max_M3508_Lim22;
        break;
        
        case 30:
            max_pulse = Pusel_Max_Lim30;
            s16shoot_speed = Shoot_Speed_Max_M3508_Lim30;
        break;
        
        default:
            max_pulse = Pusel_Max_Lim15;
            s16shoot_speed = Shoot_Speed_Max_M3508_Lim15;
        break;
    }
    
    
    Pulse_Out = max_pulse;
    
    #ifdef SHOOT_SPEED_DEBUG
        
    #endif
    
    #ifdef SHOOT_SPEED_WAVE
//        UART2_SendWave(2,4,&tar_speed,&now_speed);
    #endif
    
}

/**
  * @brief  点射时间获取
  * @param  void
  * @retval 位置环实时指令时间
  * @attention  用于发射延时测试
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}


/**
  * @brief 准备拨盘状态？（预热？按键z？）
  * @param  控制指令结构体
  * @retval void
  * @attention  
  */
float look17 = 0,look9 = 0;
void bulletPrepare(REMOTE_DATA_T RemoteMsg)
{
	static int8_t State_Pre = 0; 
	static int8_t State_Error = 0; 
	static int8_t State_Choose = 1 ;
	static uint8_t wheel;
  wheel = RemoteMsg.Wheel < -100 ? 1:0;
	look17 = wheel;
	State_Error = State_Pre - (RemoteMsg.KeyBoard.z||wheel);
	look9 = State_Error;
	if(State_Error==-1)
	{
		State_Choose=-State_Choose;
	}
	if(State_Choose<0)
	{
		lidOpen();
	}
	else
	{
		lidClose();
	}
	State_Pre=(RemoteMsg.KeyBoard.z||wheel);
	
}
/**
  * @brief  打开弹仓盖
  * @param  void
  * @retval void
  * @attention  
  */
void lidOpen(void)
{
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,LID_OPEN_PULSE);
   // Holder.Pitch.tar_angle = (MAX_PITCH_ANGLE_MOTOR - MIN_PITCH_ANGLE_MOTOR)/2 + MIN_PITCH_ANGLE_MOTOR;
	  judge_lid_state='K';
}
/**
  * @brief  关闭弹仓盖
  * @param  void
  * @retval void
  * @attention  
  */
void lidClose(void)
{
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,LID_CLOSE_PULSE);
	  judge_lid_state='G';
}


void Shoot_FreqControl(REMOTE_DATA_T RemoteMsg)
{
    float cur = (float)JUDGE_u16GetRemoteHeat17();
    float lim = (float)JUDGE_u16GetHeatLimit();
    float rate = (float)JUDGE_u16GetHeatRate();
    float buf = lim - cur;
    
    s16shoot_freq = (buf/lim)*rate * 50;

    s16shoot_freq = Constrain_Uint16_t(s16shoot_freq, Freq_Min, Freq_Max);
    
    if (buf<Buf_Safety)
    {
        s16shoot_freq = 25;
    }
    
    if (RemoteMsg.S2 == 2 || global_vision_state == VISION_WINDMILL)
    {
        s16shoot_freq = 50; /*< 大风车模式降低射频，单发 */
    }
    
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/


