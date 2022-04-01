/**
  ******************************************************************************
  * @file    holder.c
  * @author  sy xl qj
  * @brief   ��̨����
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
  * ��̨ע�����
  * 1.�ȿ��������Ƿ���ڻ�е����
  * 2.���㺯�������Զ��壬��Ҫ��math.h��
  ******************************************************************************
  */

/*includes ------------------------------------------------------------------*/
#include "holder_config.h"
#include "config.h"
#include <string.h>
#include "vision.h"
#include "holder.h"
#include "motor.h"
#include "usart.h"
#include "judge.h"
#include "can.h"
#include "tim.h"
#include "kfilter.h"
#include "ADRC.h"
#include "chassis.h"
#include "fytpi_math.h"

/* typedef -------------------------------------------------------------------*/
typedef enum
{
    cruies_e = 0,
    track_e
}GIMBAL_STATE_E;
/* define --------------------------------------------------------------------*/
#define VISION (1) /*< �Ӿ�ģʽ */
#define MANUAL (0) /*< �ֶ�ģʽ */

/* variables -----------------------------------------------------------------*/
HOLDER_T Holder;
static uint8_t holder_ctrl_state; /*< ��̨����ģʽ �ֶ����Ӿ�*/
int32_t x = 0; /*< ����ƶ��ٶȣ�ע�����ٶȣ����ֺ����λ��*/
int32_t y = 0;

int32_t vision_y_bias = 4000;
int32_t vision_x_bias = 0;
static int32_t y_bias = 0;
/* function ------------------------------------------------------------------*/

/**
  * @brief  ��̨PID��ʼ��
  * @param  void
  * @retval void
  * @attention
  */
static void holderPidInit(void)
{
    /* Yaw �����Ƿ���*/
    pidAbsoluteInit(&Holder.Yaw.PidAbsAngle, yaw_angle_min_kp, yaw_angle_min_ki, yaw_angle_kd,
                      yaw_angle_ki_limit, yaw_angle_kp_limit);
    pidAbsoluteInit(&Holder.Yaw.PidAbsSpeed, yaw_speed_kp, yaw_speed_ki, yaw_speed_kd,
                      yaw_speed_ki_limit, yaw_speed_kp_limit);
    
    /* Pitch �������*/
    pidAbsoluteInit(&Holder.Pitch.PidAbsAngle, pitch_angle_min_kp, pitch_angle_min_ki, pitch_angle_kd,
                      pitch_angle_ki_limit, pitch_angle_kp_limit);
    pidAbsoluteInit(&Holder.Pitch.PidAbsSpeed, pitch_speed_kp, pitch_speed_ki, pitch_speed_kd,
                      pitch_speed_ki_limit, pitch_speed_kp_limit);
    
    #if defined(ADRC)
    Holder_Adrc_Init();
    
    #endif
}

/**
  * @brief  ��̨PID���ƣ��ֶ�����
  * @param  void
  * @retval void
  * @attention ���ʧ�أ������ͨ�˲����ͣ�Ч������
  */
static void Holder_Pid_Manual(void)
{
    static uint8_t init_state = 0;

    if (init_state == 0) /*< δ��ʼ�� */
    {
        holderPidInit();
        init_state = 1;
    }
    else if (init_state == 1)/*< �ѳ�ʼ�� */
    {
        holderPidInit();
        /* ��ṹPI */
        piTunningAbsolute(&Holder.Yaw.PidAbsAngle, yaw_angle_min_kp, yaw_angle_tunning_kp, 
                                   yaw_angle_min_ki, yaw_angle_tunning_ki,
                                   yaw_angle_kd);
        
        Holder.Yaw.tar_speed_lpf = pidAbsoluteUpdate(Holder.Yaw.tar_angle,Holder.Yaw.angle,&Holder.Yaw.PidAbsAngle);
        Holder.Yaw.tar_speed = yaw_angle_lpf_bias * Holder.Yaw.tar_speed_lpf + (1-yaw_angle_lpf_bias) * Holder.Yaw.tar_speed;
        Holder.Yaw.output_lpf = pidAbsoluteUpdate(Holder.Yaw.tar_speed,Holder.Yaw.speed,&Holder.Yaw.PidAbsSpeed);
        Holder.Yaw.output = yaw_speed_lpf_bias * Holder.Yaw.output_lpf + (1-yaw_speed_lpf_bias) * Holder.Yaw.output;
        Holder.Yaw.output = constrainInt32(Holder.Yaw.output, -yaw_output_limit, yaw_output_limit);

        /* ��ṹPI */
        piTunningAbsolute(&Holder.Pitch.PidAbsAngle, pitch_angle_min_kp, pitch_angle_tunning_kp, 
                                   pitch_angle_min_ki, pitch_angle_tunning_ki,
                                   pitch_angle_kd);
        
        Holder.Pitch.tar_speed_lpf = pidAbsoluteUpdate(Holder.Pitch.tar_angle,Holder.Pitch.angle,&Holder.Pitch.PidAbsAngle);
        Holder.Pitch.tar_speed = pitch_angle_lpf_bias * Holder.Pitch.tar_speed_lpf + (1-pitch_angle_lpf_bias) * Holder.Pitch.tar_speed;
        Holder.Pitch.output_lpf = pidAbsoluteUpdate(Holder.Pitch.tar_speed,Holder.Pitch.speed,&Holder.Pitch.PidAbsSpeed);
        Holder.Pitch.output = pitch_speed_lpf_bias * Holder.Pitch.output_lpf + (1-pitch_speed_lpf_bias) * Holder.Pitch.output;
        Holder.Pitch.output = constrainInt32(Holder.Pitch.output, -pitch_output_limit, pitch_output_limit);  
    }
}

void Holder_RosControl(REMOTE_DATA_T RemoteMsg)
{
    switch (RecvData.gimbal_state)
    {
        case cruies_e:
            x= RemoteMsg.Mouse_x;
            y= RemoteMsg.Mouse_y * 0.15f;
            y = constrainInt32(y, -key_yaw_limit, key_yaw_limit);
            Holder.Yaw.tar_angle -= key_yaw_bias*x;
            Holder.Pitch.tar_angle -=  key_pitch_bias*y;
        break;

        case track_e:
            Holder.Yaw.tar_angle   += RecvData.vel_yaw*0.027f;
            Holder.Pitch.tar_angle += RecvData.vel_pitch*0.01f;
        break;

        default:
        break;
    }
    
}

/**
  * @brief  ��̨PID����
  * @param  void
  * @retval void
  * @attention ���ֿ�������أ�Ч�������ر��
  *            vision������Ŀ����ͼ�����ĵ�ƫ��ֵ
  */
static void Holder_PidRun(REMOTE_DATA_T RemoteMsg)
{
	/* ���� */
	//#define VISION_NO_CP
    #ifdef VISION_NO_CP
    holder_ctrl_state = VISION;
    #endif
    
    Holder_VisionPrediction();
    
	if (holder_ctrl_state == VISION)
	{
        Holder_RosControl(RemoteMsg);
        Holder_Pid_Manual();
	}
	/* �ֶ� */
	else if (holder_ctrl_state == MANUAL)
	{
        Holder_Pid_Manual();
	}
}

/**
  * @brief  ��ȡ��̨��������
  * @param  ������Ϣ�ṹ��
  * @retval void
  * @attention ��ʱֻ�м��̿��ƣ����ڼ���
  */

static void Holder_GetMoveData(REMOTE_DATA_T RemoteMsg)
{
//    static uint8_t vision_init = 0;
    /* ң�������� */
    if (RemoteMsg.S1 == SUP || RemoteMsg.S1 == SDOWN)
    {
        /* ң�������� */
        Holder.Pitch.tar_angle += rc_pitch_bias * RemoteMsg.Ch1;
        
        switch(RemoteMsg.S2)
        {
            case SUP:
							 // Holder.Pitch.tar_angle += rc_pitch_bias *RemoteMsg.Ch1;
            case SMID:
                Holder.Yaw.tar_angle   += rc_yaw_bias * RemoteMsg.Ch0;
            break;
            
            case SDOWN:
//                Holder.Yaw.tar_angle   -= getDirAngleFloat(); /*< ��̨�������*/
                Holder_RosControl(RemoteMsg);
                Holder.Yaw.tar_angle -= 0.5f * RecvData.vel_right; /*< ��̨��������ٶ�ʸ������ */
            break;
            
            default:
                Holder.Yaw.tar_angle   -= rc_yaw_bias * RemoteMsg.Ch0;
            break;   
        }
		
    }
    else
    {
        /* ��ס����Ҽ������Ӿ����� */
        if (RemoteMsg.MouseClick_right == 1) /*<  && vData.Pos.z != -1 */
        {
            holder_ctrl_state = VISION; /*< ��������״̬*/
            y= RemoteMsg.Mouse_y;
            x= RemoteMsg.Mouse_x;
        }
        else
        {
            holder_ctrl_state = MANUAL; /*< �����ֶ�״̬*/
            
            x= RemoteMsg.Mouse_x;
            y= RemoteMsg.Mouse_y * 0.2f;
            y = constrainInt32(y, -key_yaw_limit, key_yaw_limit);
            Holder.Yaw.tar_angle -= key_yaw_bias*x;
            Holder.Pitch.tar_angle -=  key_pitch_bias*y;
        }
    }
	
	/* �Ƕȱ��� */
    Holder.Pitch.tar_angle = constrainInt32(Holder.Pitch.tar_angle,MIN_PITCH_ANGLE_MOTOR,MAX_PITCH_ANGLE_MOTOR);
}


/**
  * @brief  ��ȡ��̨״̬����
  * @param  ��̨״̬�ṹ��
  * @retval void
  * @attention ����Pitch�Ƕȷ�Χ (-540, 540)
  */

static void Holder_MsgIn(HOLDER_DATA_T HolderMsg)
{   
    /* �����Ƿ��� */
    Holder.Yaw.angle = HolderMsg.angle[2]*200.0f; 
    Holder.Yaw.speed = HolderMsg.speed[2]*200.0f;        

		
    Holder.Pitch.angle = Holder.Pitch._0x209.Rx.angle*10;

    Holder.Pitch.speed = Holder.Pitch._0x209.Rx.speed;
    
    Holder.Pitch.imu_angle = HolderMsg.angle[0]*100.0f;
}

/**
  * @brief  ��̨������ʼ��
  * @param  ��̨״̬�ṹ��
  * @retval void
  * @attention
  */
void Holder_Init(HOLDER_DATA_T HolderMsg)
{
    Holder_MsgIn(HolderMsg);
	  Holder_Reset();
}

/**
  * @brief  ��̨���can����
  * @param  void
  * @retval void
  * @attention
  */
void Holder_CanTransmit(void)
{   
    if(Observer.Tx.DR16_Rate>15) /*< ң����������������16ʱ�ſ������� */
    {
        Holder.can_data[0] = Holder.Pitch.output>>8;
        Holder.can_data[1] = Holder.Pitch.output;        
        Holder.can_data[2] = Holder.Yaw.output>>8;
        Holder.can_data[3] = Holder.Yaw.output;
    }
    else
    {
        Holder_Reset(); /*< �ر�ң��������̨Ŀ��Ƕ�һֱ���ֵ�ǰ״̬ */
    }
    CAN1_Transmit(0x2FF,Holder.can_data);
}

/**
  * @brief  ��̨����
  * @param  ��̨״̬ ��̨����
  * @retval void
  * @attention Ŀǰ�Ӿ�ʶ����ɫ��Ҫ�ĳ��򣬺��ڸĽ�
  */
void Holder_Process(HOLDER_DATA_T HolderMsg,REMOTE_DATA_T RemoteMsg)
{
//    Holder_Debug();
	  Holder_MsgIn(HolderMsg);
    Holder_GetMoveData(RemoteMsg);
    Holder_Protect(RemoteMsg); 

    Holder_PidRun(RemoteMsg);
}

/************************************��̨��������*********************************/
/**
  * @brief  ��̨PID������㣬�Ƕȱ��֣�����Ϊ��ǰ�Ƕȣ�
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Reset(void)
{
    Holder.Yaw.tar_angle   = Holder.Yaw.angle;
	  Holder.Pitch.tar_angle = Holder.Pitch.angle;
    Holder.Pitch.PidAbsAngle.ctr_out = 0;
    Holder.Pitch.PidAbsAngle.err_d   = 0;
    Holder.Pitch.PidAbsAngle.err_i   = 0;
    Holder.Pitch.PidAbsSpeed.ctr_out = 0;
    Holder.Pitch.PidAbsAngle.err_d   = 0;
    Holder.Pitch.PidAbsAngle.err_i   = 0;
    Holder.Pitch.PidAbsSpeed.err_i   = 0;
    memset(Holder.can_data,0,sizeof(Holder.can_data)); /*< ���CAN */
}


/**
  * @brief  ��̨�Ƕȱ�������ֹ��ת
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Protect(REMOTE_DATA_T RemoteMsg)
{ 
    /*��V��һ���ֶ���λ������Ҳ��λ������������㣩 */
    if (RemoteMsg.KeyBoard.v == 1)
    {
        Holder_Reset();
    }
}

/**
  * @brief  ����Ԥ��ֵ����
  * @param  void
  * @retval void
  * @attention ��̨��������ٶ� - Ŀ�������̨�ٶ� = Ŀ������ٶȣ���Ե����ٶȣ�
  */
int32_t vision_holder_speed = 0;
int32_t vision_armor_speed_to_holder = 0;
int32_t vision_armor_speed = 0;
int32_t vData_old = 0;
int32_t vData_now = 0;
void Holder_VisionPrediction(void)
{
    vData_now = vData_x;
    vision_holder_speed = Holder.Yaw.speed;
    vision_armor_speed_to_holder = vData_now - vData_old;
    vData_old = vData_now;
    
    vision_armor_speed = vision_holder_speed - vision_armor_speed_to_holder;
    
    vision_x_bias = vision_armor_speed;
}


void Holder_Debug()
{
    #ifdef HOLDER_DEBUG
        holderPidInit();
    
    #if defined(INFANTRY_6)
        //Holder_Adrc_Init();
    #endif
    
    #endif
    
    #ifdef HOLDER_PITCH_WAVE
//        UART2_SendWave(4, 4, &Holder.Pitch.tar_angle, &Holder.Pitch.angle, 
//                            &Holder.Pitch.tar_speed, &Holder.Pitch.speed);//&Holder.Pitch.output);
    
//    int32_t y = ADRC_Pitch_Speed_Controller.x1;
//    UART2_SendWave(2, 4, &Holder.Pitch.tar_speed, &y);
    #endif
    
    #ifdef HOLDER_YAW_WAVE
        UART2_SendWave(5, 4, &Holder.Yaw.tar_angle, &Holder.Yaw.angle, 
                            &Holder.Yaw.tar_speed, &Holder.Yaw.speed, &Holder.Yaw.output);
    #endif
    
    #ifdef HOLDER_VISION_WAVE
        UART2_SendWave(3, 2, &vData_x, &vData_y, &vData_z);
    #endif
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
