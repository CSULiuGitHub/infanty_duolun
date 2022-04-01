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
#include "misc_func.h"
#include "vision.h"
#include "config.h"
#include "string.h"
#include "holder.h"
#include "judge.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "can.h"
#include "message.h"
#include "judge_rx.h"
#include <math.h>
/* typedef -------------------------------------------------------------------*/
typedef struct _ChassisSpeed
{
    int16_t x;
    int16_t y;
    int16_t yaw;
}CHASSIS_SPEED_T;

typedef enum
{
    manual_e = 0,
    armor_e,
    small_windmill_e,
    big_windmill_e
}VISION_STATE_E;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
vData_t vData = {0};
CHASSIS_SPEED_T ChassisSpeed = {0};
RECV_DATA_T RecvData = {0};
uint8_t global_vision_state = VISION_MANUAL;
/* function ------------------------------------------------------------------*/

float vData_x_RMF4[4];
float vData_y_RMF4[4];
float vData_z_RMF4[4];
int16_t vData_x;
int16_t vData_y;
int16_t vData_z;
uint8_t vData_buf[16];
uint8_t tran_st=0;

#define CHEK_SEND (1)
#define CHEK_RECV (0)
uint8_t chekSum(uint8_t count_number, uint8_t *data_)
{
    uint8_t chek_sum = 0;
    uint8_t i;
    for (i=0; i<count_number; i++)
    {
        chek_sum = chek_sum^data_[i];
    }
    return chek_sum;
}

void sendCBData(void)
{
    uint8_t buf_[9];
    buf_[0] = 0x7B;
    buf_[1] = ChassisSpeed.x >> 8;
    buf_[2] = ChassisSpeed.x & 0x0F;
    buf_[3] = ChassisSpeed.y >> 8;
    buf_[4] = ChassisSpeed.y & 0x0F;
    buf_[5] = JUDGE_u8GetCommandKeyboard();
    buf_[6] = armor_e;
    buf_[7] = chekSum(7, buf_);
    buf_[8] = 0X7D;
    
    HAL_UART_Transmit_IT(&huart6, buf_, sizeof(buf_));
}

void recvCmdData(uint8_t data)
{
    static uint8_t count = 0;
    int16_t tmp_data = 0;
    sendCBData();
    
    RecvData.rx_buf[count] = data;
    if ((RecvData.rx_buf[count]>>4) == 0x0A || count>0)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    
    if (count == RECV_DATA_SIZE)
    {
        count = 0; /*< 重新开始接收 */
        if ((RecvData.rx_buf[RECV_DATA_SIZE-1]&0x0F) == 0x05)
        {
            /**
             *tips:
             *  1. "&"按位与，“|”按位或
             *  2. 0x0F = 0000 1111, 0x03 = 0000 0011
             *  3. data & 0x0F 结果是data数据前4位为0，后4位不变，以达到只保留对应位数的目的
             *  4. "&"进行删减， "|"进行拼接
             */
            tmp_data |= RecvData.rx_buf[0]&0x0F;
            RecvData.vel_yaw = (tmp_data<<7 | RecvData.rx_buf[1]>>1) - 1023;
            
            tmp_data = 0;
            tmp_data |= RecvData.rx_buf[1]&0x01;
            RecvData.vel_pitch = (tmp_data<<10 | RecvData.rx_buf[2]<<2 | RecvData.rx_buf[3]>>6) - 1023;
            
            tmp_data = 0;
            tmp_data |= RecvData.rx_buf[3]&0x3F;
            RecvData.vel_right = (tmp_data<<5 | RecvData.rx_buf[4]>>3) - 1023;
            
            tmp_data = 0;
            tmp_data |= RecvData.rx_buf[4]&0x07;
            RecvData.vel_front = (tmp_data<<8 | RecvData.rx_buf[5]) - 1023;
            
            RecvData.chassis_state = RecvData.rx_buf[6]>>6;
            RecvData.gimbal_state  = (RecvData.rx_buf[6]>>4)&0x03;
        }
    }
}

#if defined(INFANTRY_4) || defined(INFANTRY_3) || defined(INFANTRY_6) 
void Vision_RecvData(uint8_t byte)
{
    vData.buf[vData.index] = byte;
    if (vData.buf[vData.index] == 0x03 && vData.buf[vData.index - 1] == 0xFC)
    {
        if (vData.buf[vData.index - 15] == 0X03 && vData.buf[vData.index - 14] == 0XFC)
        {
            memcpy(&vData.Pos, &vData.buf[vData.index - 13], 12);
            vData.index = 0;
			tran_st=1;
        }else if(vData.buf[vData.index - 5] == 0X03 && vData.buf[vData.index - 4] == 0XFC)
        {
            uint8_t robot_id = vData.buf[vData.index - 3];
			uint8_t enemy_color = vData.buf[vData.index - 2];
			vData.index = 0;
			tran_st=0;
        }
    }
    
    vData_x = vData.Pos.x * 100*10;
    vData_y = vData.Pos.y * 100*10;
    vData_z = vData.Pos.z * 100;
    
    vData.index++;
    if (vData.index == 200)
        vData.index = 0;
}
#endif

#if defined(INFANTRY_5)
void Vision_RecvData(uint8_t byte)
{
    static uint8_t count = 0;
    
    vData_buf[count] = byte;
    if (vData_buf[count] == 0x7b || count>0)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    
    if (count == 8)
    {
        count = 0; /*< 重新开始接收 */
        if (vData_buf[7] == 0x7d)
        {
            vData_x = ((vData_buf[1]<<8) + vData_buf[2])*10;
            vData_y = ((vData_buf[3]<<8) + vData_buf[4])*10;  
            vData_z = ((vData_buf[5]<<8) + vData_buf[6]);  
        }
    }
}
#endif

void Vision_SendData(REMOTE_DATA_T RemoteMsg)
{
    uint8_t tmp_data[12]; 
	  static uint16_t tx_tick = 0;
    static uint8_t delay_tick = 0;
    static uint8_t color_state  = 0; 
    static uint8_t detect_state = 0;
    uint8_t id = (uint8_t)JUDGE_u8GetRobotId();
    
    if (id <= 7 && id >0)
    {
        color_state = 1;
    }
    else if (id > 7)
    {
        color_state = 0;
    }
    
    if (delay_tick < 10)
    {
        delay_tick++;
    }
    
    if (RemoteMsg.KeyBoard.v == 1 && detect_state == 0 && delay_tick == 10)
    {
        detect_state = 1;
        delay_tick = 0;
    }
    else if (RemoteMsg.KeyBoard.v == 1 && detect_state == 1 && delay_tick == 10)
    {
        detect_state = 0;
        delay_tick = 0;
    }
    
    tmp_data[2] = 0x03; /*< 机器人ID */
    switch(color_state)
    {
        case 0:
            tmp_data[3] = 0x00; /*< 红色*/
        break;

        case 1:
            tmp_data[3] = 0x01; /*< 蓝色*/
        break;
        
        default:
            tmp_data[3] = 0x00; /*< 红色*/
        break;
    }
    
    
    switch(detect_state)
    {
        case 0:
            tmp_data[4] = 0x00; /*< 自瞄*/
            global_vision_state = VISION_AUTO;
	 	    judge_vision_state = 'A';
        break;

        case 1:
            tmp_data[4] = 0x01; /*< 小符*/
            global_vision_state = VISION_WINDMILL;
		    judge_vision_state = 'W';
        break;
        
        default:
            tmp_data[4] = 0x00; /*< 自瞄*/
            global_vision_state = VISION_AUTO;
		    judge_vision_state = 'A';
        break;
    }
    
    tmp_data[5] = (uint8_t)JUDGE_fGetSpeedHeat17();
    tmp_data[6] = Holder.Yaw._0x20A.Mc.angle >> 8;
    tmp_data[7] = Holder.Yaw._0x20A.Mc.angle;
    tmp_data[8] = Holder.Pitch._0x209.Mc.angle >> 8;
    tmp_data[9] = Holder.Pitch._0x209.Mc.angle;
    

    /* 帧头帧尾 */
    tmp_data[0] = 0x03;
    tmp_data[1] = 0xFC;

    tmp_data[10] = 0xFC;
    tmp_data[11] = 0x03;
	tx_tick++;
	if(tx_tick > 20)
	{
		HAL_UART_Transmit_IT(&huart6, tmp_data, 12);
		tx_tick = 0;
	}
    osDelay(1);
}



void getChassisSpeed(void)
{
    int32_t tmp_speed_ = 0.5 * (Chassis.m3508[0].Rx.speed + (-Chassis.m3508[1].Rx.speed));
    float tmp_angle_ = Chassis.m6020[0].Mc.angle / DEG8192 + getDirAngleFloat();
    
    ChassisSpeed.x = tmp_speed_ * cosf(tmp_angle_ / RAD2DEG);
    ChassisSpeed.y = -tmp_speed_ * sinf(tmp_angle_ / RAD2DEG);
    ChassisSpeed.yaw = getImuYaw();
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
