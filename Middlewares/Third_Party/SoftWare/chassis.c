/**
  ******************************************************************************
  * @file    chassis.c
  * @author  施阳 胡小璐 宋其津 曾俊杰 周晓佳
  * @brief   底盘控制
  * @date    2022-01-30
  ******************************************************************************
  * @attention 无
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by fyt under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *修改记录：
  *<时间>      |<版本>      |<作者>      |<描述>     
  *2022-02-08  |v2.0        |qj          |首次发布
  ******************************************************************************
**/
  
/* includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include "chassis_config.h"
#include "chassis.h"
#include "can.h"
#include "holder.h"
#include "usart.h"
#include "ui.h"
#include "tim.h"
#include "fytpi_math.h"
#include "judge.h"
#include "judge_tx.h"
#include "vision.h"

/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define FOLLOW 1
#define Radius 0.2446f    //机械人中心到舵轮的距离
#define L      1.0f      //机械人中心到舵轮的距离

#define RUDDER_ANGLE_0 6381
#define RUDDER_ANGLE_1 5207

#define SPIN    5000     //旋转速度
#define CHASSIS_FOLLOW (0)
#define CHASSIS_SPIN   (1)
#define PI 3.141593f

/* variables -----------------------------------------------------------------*/
CHASSIS_T Chassis = {0};

static int16_t front_temp,right_temp;
static int16_t s16_max_front_speed = 320; /* 默认最大前后移动速度 */
static int16_t s16_max_right_speed = 320; /* 默认最大左右移动速度 */

static uint8_t chassis_state = CHASSIS_FOLLOW;

static float fmax_speed_spin  = 8000.0f;  /*< 最大旋转速度（限幅用）*/
static float fmax_wheel_speed = 2000.0f;

/*速度功率控制（需实测）*/
static float spin_speed[8]  = {3100.0f, 8000.0f, 1000.0f, 1200.0f, 1350.0f, 1500.0f,
                                1750.0f, 1900.0f};
static float wheel_speed[8] = {10000.0f, 10000.0f, 10000.0f, 10000.0f, 10000.0f,
                                10000.0f, 10000.0f, 10000.0f};

/* function ------------------------------------------------------------------*/
/**
  * @brief  底盘模式选择（r键选择？）
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
float look3 = 0;
static void Chassis_ChooseMode(REMOTE_DATA_T RemoteMsg)
{
    int32_t spin_s16temp=0;
    static uint8_t delay_tick;
    
    float dir_angle = getDirAngleFloat();
    switch(RemoteMsg.S1)
    {
        case SUP:
			remoteControlGetMoveData(RemoteMsg);
      omniSpeedParse(dir_angle, front_temp, right_temp);
            switch(RemoteMsg.S2)
            {
                case SMID:  /*< 跟随速度 */
                    spin_s16temp = followSpeedSpinS16(Chassis.MoveData.right); 
                break;
                
                case SUP:   /*< 跟随云台 */
                    spin_s16temp = followHolderSpinS16(dir_angle);
                break;
                
                case SDOWN: /*< 云台跟随底盘*/
                    spin_s16temp = holderFollowGetMoveDataS16(RemoteMsg); 
                break;
                
                default:    /*< 跟随云台 */
                    spin_s16temp = followHolderSpinS16(dir_angle); 
                break;
            }
        break; /*< end case SUP */
            
        case SDOWN:
            remoteControlGetMoveData(RemoteMsg);
            omniSpeedParse(dir_angle, front_temp, right_temp);
            switch(RemoteMsg.S2)
            {
                case SMID: /*< 旋转 */
                     spin_s16temp = dynamicSpin(); 
                break;
                
                case SUP: /*< 旋转 */
                     spin_s16temp = dynamicSpin();
                break;
                
                case SDOWN: /*< ROS控制*/
                     spin_s16temp = Chassis_RosControl(dir_angle);
                break;
                
                default: /*< 旋转 */
                     spin_s16temp = dynamicSpin(); 
                break;
            }
            
        break; /*< end case SDOWN */
        
        case SMID: /*< 键鼠模式 */
            keyboardControlGetMoveData(RemoteMsg);
            omniSpeedParse(dir_angle, front_temp, right_temp);
            if (delay_tick < 50)
            {
                delay_tick++;
            }
            
            /* R键切换跟随和小陀螺模式 */
           if (RemoteMsg.KeyBoard.r == 1 && chassis_state == CHASSIS_FOLLOW && delay_tick == 50)
            {
                chassis_state = CHASSIS_SPIN;
                delay_tick = 0;
            }
            else if (RemoteMsg.KeyBoard.r == 1 && chassis_state == CHASSIS_SPIN && delay_tick == 50)
            {
                chassis_state = CHASSIS_FOLLOW;
                delay_tick = 0;
            }
            
            switch(chassis_state)
            {
                case CHASSIS_FOLLOW:
                    if (RemoteMsg.KeyBoard.c == 1)
                    {
                        spin_s16temp = followSpeedSpinS16(Chassis.MoveData.right);
                    }
                    else
                    {
                        spin_s16temp = followSpeedSpinS16(Chassis.MoveData.right);
                    }
                break;
                    
                case CHASSIS_SPIN:
                    if (RemoteMsg.KeyBoard.c == 1)
                    {
//											  spin_s16temp = dynamicSpin();
                        spin_s16temp = followHolderSpinS16(dir_angle);
                    }
                    else 
                    {
                        spin_s16temp = dynamicSpin();
                    }
                break;
                    
                default:
                break;
            } /*< end switch chassis state */
        break; /*< end case SMID */
            
        default:
        break;        
    } /*< end switch S1 */

    Chassis.MoveData.clock_wise = constrainInt32(spin_s16temp, -60000, 60000);
}

/**
  * @brief  电机速度控制
  * @param  void
  * @retval void
  * @attention 
  */
float tmp_angle[2];
float tmp_speed[2];
float speed_angle[2];
static void Chassis_SpeedControl(REMOTE_DATA_T RemoteMsg)
{
    uint8_t i;
    float limit_speed = fmax_wheel_speed;
    float tmp_max_speed = 1.0f;
    float tmp_min_speed = 1.0f;
    float cmp_index = 1.0f;
    float vector[2] = {1.0f, 1.0f};
		//舵向速度
    speed_angle[0] = atan2(Chassis.MoveData.right - Chassis.MoveData.clock_wise * Radius * fastSin(45),
                           Chassis.MoveData.front - Chassis.MoveData.clock_wise * Radius * fastCos(45));
    
    speed_angle[1] = atan2(Chassis.MoveData.right + Chassis.MoveData.clock_wise * Radius * fastSin(45),
                           Chassis.MoveData.front + Chassis.MoveData.clock_wise * Radius * fastCos(45));
		
    tmp_angle[0]   = RUDDER_ANGLE_0 + (speed_angle[0]) * RAD2DEG * DEG8192;
    tmp_angle[1]   = RUDDER_ANGLE_1 + (speed_angle[1]) * RAD2DEG * DEG8192;
		
			for(i=0; i<2; i++)
			{
					int error = tmp_angle[i] - Chassis.m6020[i].angle + Chassis.m6020[i].Mc.circle*8192;
					if(error > 270 * DEG8192)
					{
							error -= 2*PI* RAD2DEG * DEG8192;
					}
					else if(error > PI/2* RAD2DEG * DEG8192)
					{
							error -= PI* RAD2DEG * DEG8192;
							vector[i] = -1;
					}
					else if(error < -270 * DEG8192)
					{
							error += 2*PI* RAD2DEG * DEG8192;
					}
					else if(error < -PI/2* RAD2DEG * DEG8192)
					{
							error += PI* RAD2DEG * DEG8192;
							vector[i] = -1;
					}
					
					tmp_angle[i] = Chassis.m6020[i].angle + error;
			}
    
    //驱动轮速度
		tmp_speed[0]  =    vector[0]*(sqrt( pow(Chassis.MoveData.right - Chassis.MoveData.clock_wise*Radius*fastCos(45),2)
														      +    pow(Chassis.MoveData.front - Chassis.MoveData.clock_wise*Radius*fastSin(45),2)));
		tmp_speed[1]  =    vector[1]*(sqrt( pow(Chassis.MoveData.right + Chassis.MoveData.clock_wise*Radius*fastCos(45),2)
														      +    pow(Chassis.MoveData.front + Chassis.MoveData.clock_wise*Radius*fastSin(45),2)));
			
    for(i = 0; i<2; i++)
    {
        /* 找最大值 */
        if (tmp_speed[i] > tmp_max_speed)
        {
            tmp_max_speed = tmp_speed[i];
        }
        
        /* 找最小值 */
        if (tmp_speed[i] < tmp_min_speed)
        {
            tmp_min_speed = tmp_speed[i];
        }
    }

    tmp_max_speed = absFloat(tmp_max_speed);
    tmp_min_speed = absFloat(tmp_min_speed);

    if (tmp_max_speed > limit_speed || tmp_min_speed > limit_speed)
    {
        if (tmp_max_speed < tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
        else if (tmp_max_speed > tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_max_speed;
        }
        else
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
    }
    else
    {
        cmp_index = 1;
    }

    for(i = 0; i<2; i++)
    {
        Chassis.m3508[i].tar_speed = (int16_t)(tmp_speed[i]*cmp_index) * 6;
    }
    
    for(i = 0; i<2; i++)
    {
        Chassis.m6020[i].tar_angle= (tmp_angle[i]);
    }
}

/**
  * @brief  底盘PID输出
  * @param  void
  * @retval void
  * @attention
  */
static void Chassis_PidRun(void)
{
    uint8_t i;
    #ifdef CHASSIS_WAVE
//         UART2_SendWave(4, 2, &Chassis.m3508[0].tar_speed,  &Chassis.m3508[0].Rx.speed,
//                              &Chassis.m3508[0].tar_current,&Chassis.m3508[0].Rx.current);
         UART2_SendWave(2, 2, &Chassis.m6020[0].tar_angle, Chassis.m6020[0].angle);
    
    #endif

    #ifdef CHASSIS_DEBUG
           Chassis_Init();
    #endif

					
	
    for (i = 0; i < 2 ;i++)     //rx speed lpf
    {
        Chassis.m3508[i].LPF.speed 
        = 0.8 * Chassis.m3508[i].Rx.speed 
        + 0.2 * Chassis.m3508[i].LPF.speed;
    }   
    for (i = 0; i < 2; i++)     //speed loop
    {
        Chassis.m3508[i].tar_current 
        = pidIncrementUpdate(Chassis.m3508[i].tar_speed, 
                             Chassis.m3508[i].LPF.speed,
                            &Chassis.m3508[i].PidIncSpeed);
    }
    for (i = 0; i < 2; i++)     //tar current lpf 
    {
        Chassis.m3508[i].LPF.tar_current 
        = 0.8 * Chassis.m3508[i].tar_current 
        + 0.2 * Chassis.m3508[i].LPF.tar_current;
    }
    
    for (i = 0; i < 2; i++)     //rx current lpf 
    {
        Chassis.m3508[i].LPF.current 
        = 0.8 * Chassis.m3508[i].Rx.current 
        + 0.2 * Chassis.m3508[i].LPF.current;
    }
    
    for (i = 0; i < 2; i++)     //current loop
    {
        Chassis.m3508[i].output
        = pidIncrementUpdate(Chassis.m3508[i].LPF.tar_current,
                             Chassis.m3508[i].LPF.current,
                            &Chassis.m3508[i].PidIncCurrent);
    }

    for (i = 0; i < 2; i++)     //out lpf
    {
        Chassis.m3508[i].LPF.output 
        = 0.8 * Chassis.m3508[i].output
        + 0.2 * Chassis.m3508[i].LPF.output;
    }

    for (i = 0; i < 2; i++)     //tar speed
    {
        Chassis.m6020[i].tar_speed
        = pidAbsoluteUpdate(Chassis.m6020[i].tar_angle,
                            Chassis.m6020[i].angle,
                            &Chassis.m6020[i].PidAbsAngle);
    }

    for (i = 0; i < 2; i++)     //out
    {
        Chassis.m6020[i].output 
        = pidAbsoluteUpdate(Chassis.m6020[i].tar_speed, 
                            Chassis.m6020[i].Rx.speed, 
                            &Chassis.m6020[i].PidAbsSpeed);
    }
}


/**
  * @brief  超级电容CAN指令发送
  * @param  void
  * @retval void
  * @attention 
  */
static void superCapCanTransmit(void)
{
    Chassis.SuperCap.can_data[0] = Chassis.SuperCap.target_power >> 8;
    Chassis.SuperCap.can_data[1] = Chassis.SuperCap.target_power;
    CAN2_Transmit(0x210,Chassis.SuperCap.can_data);
}

/**
  * @brief  功率控制，向超级电容发送最大功率
  * @param  void
  * @retval void
  * @attention 超级电容根据最大功率控制电流输出，防止底盘超功率
  */
float look11,look7;
static void Chassis_Powerlimit(void)
{
    static uint8_t tick = 0;
    int16_t max_move_speed;
    int16_t max_spin_speed;

    uint16_t power_lim = JUDGE_u16GetChassisPowerLimit();
    uint16_t power_buf = JUDGE_u16GetRemainEnergy();
    float speed_bias = 10*(float)(Chassis.SuperCap.cap_vol)
                     / (float)JUDGE_u16GetChassisVolt();
//    float speed_bias = 1;
    tick++;/*< 延时*/
    if (tick == 20)
    {
        if (power_lim < 50)
        {
            power_lim = 50;
        }
        
        Chassis.SuperCap.target_power = power_lim*100;
        
        Chassis.SuperCap.target_power 
        = constrainUint16(Chassis.SuperCap.target_power, 3000, 15000);
        
        if(power_buf < 15)
        {
            Chassis.SuperCap.target_power = (power_lim-5)*100;
        }
        superCapCanTransmit();
        tick = 0;
    }
    
    switch(power_lim)
    {
        case 40:
            fmax_wheel_speed = wheel_speed[0];
            fmax_speed_spin  = spin_speed[0];
            max_spin_speed = spin_speed[0];
            max_move_speed = 320;
        break;
        
        case 45:
            fmax_wheel_speed = wheel_speed[1];
            fmax_speed_spin  = spin_speed[1];
            max_spin_speed = spin_speed[1];
            max_move_speed = 330;
        break;
        
        case 50:
					  fmax_wheel_speed = wheel_speed[1];
            fmax_speed_spin  = spin_speed[1];
            max_spin_speed = 2000;//spin_speed[1];
            max_move_speed = 340;
				break;
				
        case 55:
            fmax_wheel_speed = wheel_speed[2];
            fmax_speed_spin  = spin_speed[2];
            max_spin_speed = spin_speed[2];
            max_move_speed = 350;
        break;
        
        case 60:
            fmax_wheel_speed = wheel_speed[3];
            fmax_speed_spin  = spin_speed[3];
            max_spin_speed = spin_speed[3];
            max_move_speed = 400;
        break;
        
        case 70:
            fmax_wheel_speed = wheel_speed[4];
            fmax_speed_spin  = spin_speed[4];
            max_spin_speed = spin_speed[4];
            max_move_speed = 410;
        break;
        
        case 80:
            fmax_wheel_speed = wheel_speed[5];
            fmax_speed_spin  = spin_speed[5];
            max_spin_speed = spin_speed[5];
            max_move_speed = 500;
        break;
        
        case 100:
            fmax_wheel_speed = wheel_speed[6];
            fmax_speed_spin  = spin_speed[6];
            max_spin_speed = spin_speed[6];
            max_move_speed = 550;
        break;
        
        case 120:
            fmax_wheel_speed = wheel_speed[7];
            fmax_speed_spin  = spin_speed[7];
            max_spin_speed = spin_speed[7];
            max_move_speed = 650;
        break;
        
        default:
            fmax_wheel_speed = wheel_speed[0];
            fmax_speed_spin  = spin_speed[0];
            max_spin_speed = spin_speed[0];
            max_move_speed = 510;
        break;
    }
    
    s16_max_front_speed = max_move_speed*speed_bias;
    s16_max_right_speed = max_move_speed*speed_bias;
    fmax_speed_spin     = max_spin_speed*speed_bias;
		
}

/**
  * @brief  (Q键)一键掉头，云台偏航+180度，底盘与云台差角+180度
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
static void Chassis_TurnRound(REMOTE_DATA_T RemoteMsg)
{
    static uint8_t delay_tick = 0;
    static uint8_t state = 0;
   
    if (delay_tick < 100)
    {
        delay_tick++; /*< 延时，防止连续按两次*/
        
    }
    
    if (RemoteMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 0)
    {
        Holder.Yaw.tar_angle += TURN_ROUND_ANGLE;
        delay_tick = 0;
        state = 1;
    }
    else if (RemoteMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 1)
    {
        Holder.Yaw.tar_angle += TURN_ROUND_ANGLE;
        delay_tick = 0;
        state = 0;
    }
}

/*********************************底盘辅助函数*********************************/
/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_SpeedReset(void)
{
    uint8_t i;
    
    Chassis.MoveData.front = 0;
    Chassis.MoveData.clock_wise = 0;
    Chassis.MoveData.right = 0;
    for(i = 0; i < 4; i++)
    {
        Chassis.m3508[i].tar_speed                = 0;
        Chassis.m3508[i].PidIncSpeed.err_now      = 0;
        Chassis.m3508[i].PidIncSpeed.err_old_1    = 0;
        Chassis.m3508[i].PidIncSpeed.err_old_2    = 0;
        Chassis.m3508[i].PidIncSpeed.ctr_out      = 0;
        Chassis.m3508[i].PidIncSpeed.d_ctr_out    = 0;
        Chassis.m3508[i].PidIncCurrent.err_now    = 0;
        Chassis.m3508[i].PidIncCurrent.err_old_1  = 0;
        Chassis.m3508[i].PidIncCurrent.err_old_2  = 0;
        Chassis.m3508[i].PidIncCurrent.d_ctr_out  = 0;
        Chassis.m3508[i].PidIncCurrent.ctr_out    = 0;
        Chassis.m6020[i].tar_angle                = 0;
        Chassis.m6020[i].PidAbsSpeed.ctr_out      = 0;
        Chassis.m6020[i].PidAbsSpeed.err_d        = 0;
        Chassis.m6020[i].PidAbsSpeed.err_i        = 0;
        Chassis.m6020[i].PidAbsAngle.ctr_out      = 0;
        Chassis.m6020[i].PidAbsAngle.err_d        = 0;
        Chassis.m6020[i].PidAbsAngle.err_i        = 0;	
    }
    memset(Chassis.can_data,0,sizeof(Chassis.can_data));
    memset(Chassis.can_data1,0,sizeof(Chassis.can_data1));
}

/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_Protect(REMOTE_DATA_T RemoteMsg)
{
    if (RemoteMsg.KeyBoard.v == 1)
    {
        Chassis_SpeedReset();
    }
}

/**
  * @brief  快速定点小陀螺
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_FastFixSpin(REMOTE_DATA_T RemoteMsg)
{
    if (RemoteMsg.KeyBoard.e == 1)
    {
        Chassis.MoveData.front = 0;
        Chassis.MoveData.right = 0;
        Chassis.MoveData.clock_wise = fmax_speed_spin;
    }
}

/**
  * @brief  动态小陀螺(实测：限轮子速度就可以了，没必要动态改旋转速度)
  * @param  控制指令结构体
  * @retval void
  * @attention （front 或 right速度 与 clockwise速度 成反比关系：没用）
  */
int32_t dynamicSpin(void)
{
    judge_spin_state='l';

    return SPIN ;//+ //(sqrt( pow(Chassis.MoveData.right,2)
							//	+        pow(Chassis.MoveData.front,2))) )*1.6;
}

/**
  * @brief  底盘跟随云台旋转
  * @param  底盘和云台差角
  * @retval 旋转速度
  * @attention 
  */

int16_t followHolderSpinS16(float dir_angle)
{
    int8_t s8_sign;
    
    s8_sign = dir_angle<0? -1:1;    //dir_angle
    
    judge_spin_state='f';
	
    return s8_sign*dir_angle*dir_angle;
}

/**
  * @brief  底盘跟随速度方向旋转
  * @param  横移速度
  * @retval 旋转速度
  * @attention 
  */
int16_t followSpeedSpinS16(float right_speed)
{   
    judge_spin_state='s';
    
    return 0;//2.5f*right_speed;
}

/**
  * @brief  全向底盘速度解算
  * @param  云台和底盘差角
  * @retval 
  * @attention 
  */
void omniSpeedParse(float dir_angle, int16_t s16_speed_front, int16_t s16_speed_right)
{
    Chassis.MoveData.front =
      (int16_t)(((float)s16_speed_front) * (fastCos((int16_t)dir_angle))) 
    + (int16_t)(((float)s16_speed_right) * (fastSin((int16_t)dir_angle)));
    
    Chassis.MoveData.right =
    - (int16_t)(((float)s16_speed_front) * (fastSin((int16_t)dir_angle))) 
    + (int16_t)(((float)s16_speed_right) * (fastCos((int16_t)dir_angle)));
}

/**
  * @brief  键盘控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void keyboardControlGetMoveData(REMOTE_DATA_T RemoteMsg)
{
    if(RemoteMsg.KeyBoard.w)
        front_temp = s16_max_front_speed;
    else if((!RemoteMsg.KeyBoard.w)&&(RemoteMsg.KeyBoard.s))
        front_temp =-s16_max_front_speed;
    else
        front_temp = 0;
    
    if(RemoteMsg.KeyBoard.d)
        right_temp = s16_max_right_speed;
    else if((!RemoteMsg.KeyBoard.d)&&(RemoteMsg.KeyBoard.a))
        right_temp =-s16_max_right_speed;
    else
        right_temp = 0;
    
    if(RemoteMsg.KeyBoard.ctrl)
    {
        front_temp = front_temp*2.2;
        right_temp = right_temp*2.2;
    }else if((!RemoteMsg.KeyBoard.ctrl)&&(RemoteMsg.KeyBoard.shift))
    {
        front_temp = front_temp*0.7;
        right_temp = right_temp*0.7;
    }
    else
    {
    }
}

/**
  * @brief  遥控器控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void remoteControlGetMoveData(REMOTE_DATA_T RemoteMsg)
{
    right_temp = RemoteMsg.Ch2;
    front_temp = RemoteMsg.Ch3;
}


/**
  * @brief  云台跟随底盘
  * @param  遥控器消息结构体
  * @retval 
  * @attention 不进行无头模式解算
  */
int16_t holderFollowGetMoveDataS16(REMOTE_DATA_T RemoteMsg)
{
    Chassis.MoveData.right = -RemoteMsg.Ch2;
    Chassis.MoveData.front = -RemoteMsg.Ch3;
    return RemoteMsg.Ch0; /*< 返回旋转速度*/
}

int16_t Chassis_RosControl(float dir_angle)
{
    right_temp = -RecvData.vel_right;
    front_temp = -RecvData.vel_front;
    omniSpeedParse(dir_angle, front_temp, right_temp);

    switch(RecvData.chassis_state)
    {
        case 1: /*< 跟随 */
            return followHolderSpinS16(dir_angle);
        break;
        
        case 0: /*< 不旋转 */
            return 0;
        break;
        
        case 2: /*< 小陀螺 */
            return SPIN;
        break;
    }
}

/**
  * @brief  底盘电机CAN
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
        for(i=0;i<2;i++)
        {
            /* 输出限幅 */
            Chassis.m3508[i].LPF.output 
            = constrainInt16(Chassis.m3508[i].LPF.output,
                             -chassis_out_lim, chassis_out_lim);
            
            Chassis.m6020[i].LPF.output 
            = constrainInt32(Chassis.m6020[i].output,
                             -rudder_out_limit, rudder_out_limit);
            /* CAN 赋值 */
            Chassis.can_data[2*i] = (uint8_t)(Chassis.m3508[i].LPF.output>>8);
            Chassis.can_data[2*i+1] = (uint8_t)(Chassis.m3508[i].LPF.output);
            Chassis.can_data1[2*i]  = (uint8_t)(Chassis.m6020[i].LPF.output>>8);
            Chassis.can_data1[2*i+1]= (uint8_t)(Chassis.m6020[i].LPF.output);
        }
    }
    else
    {
        Chassis_SpeedReset(); /*< 关闭遥控器后，底盘目标速度和累计误差清（0）*/
    }
    CAN2_Transmit(0x200,Chassis.can_data);
    CAN2_Transmit(0x1FF,Chassis.can_data1);
}

/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Init(void)
{
    uint8_t i;
	
    /* 3508电机增量式速度环 */
    for(i=0;i<2;i++)
    {
        pidIncrementInit(&Chassis.m3508[i].PidIncSpeed,
                         chassis_speed_kp, chassis_speed_ki, chassis_speed_kd,
                         chassis_speed_inc_lim);
    }
    
    /* 3508电机增量式电流环 */
    for(i=0;i<2;i++)
    {
        pidIncrementInit(&Chassis.m3508[i].PidIncCurrent,
                     chassis_current_kp, chassis_current_ki, chassis_current_kd,
                     chassis_current_inc_lim);
    }
    
    /* 6020电机绝对式角度环 */
    for(i=0;i<2;i++)
    {
        pidAbsoluteInit(&Chassis.m6020[i].PidAbsAngle,
                        rudder_angle_kp, rudder_angle_ki, rudder_angle_kd,
                        rudder_angle_limit, rudder_angle_limit);
    }
    
    /* 6020电机绝对式速度环 */
    for(i=0;i<2;i++)
    {
        pidAbsoluteInit(&Chassis.m6020[i].PidAbsSpeed,
                        rudder_speed_kp, rudder_speed_ki, rudder_speed_kd,
                        rudder_speed_limit,rudder_speed_limit);
    }		 
}

/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
  * @attention 保护写在CAN发送前，（V）一键复位，PID误差和输出清零
  */
void Chassis_Process(REMOTE_DATA_T RemoteMsg)
{
	  Chassis_SpeedReset();
    Chassis_Powerlimit();
	  Chassis_ChooseMode(RemoteMsg);
    Chassis_Protect(RemoteMsg);
    Chassis_TurnRound(RemoteMsg);    /*< 一键掉头*/
    Chassis_FastFixSpin(RemoteMsg); /*< （E）键快速定点小陀螺 */
    Chassis_SpeedControl(RemoteMsg);
	  Chassis_PidRun();
    
    getChassisSpeed(); /*< 速度反馈计算（上位机用） */
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
