#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  ���̿�����ز���
  * @attention ˫�ջ����� 
  *     �⻷���ٶȻ�����������ʽPID������
  *     �ڻ�������������������ʽPID������
  *     ���Է�����
  *         1.���� #define CHASSIS_DEBUG����ͨ��debug�����޸Ĳ���
  *         2.���� #define CHASSIS_WAVE�������������ز���
  *         ����ʵ�ַ�����chassis.c�е�void Chassis_Init(void)������
  *
  *         ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
/* #define CHASSIS_DEBUG */
/* #define CHASSIS_WAVE */

#define CHASSIS_DEBUG
#define CHASSIS_WAVE

/* ������ */
/* �ǶȻ� */
float rudder_angle_kp   = 0.25f; /*< �ǶȻ�P����12 */
float rudder_angle_ki   = 0.0f; /*< �ǶȻ�I���� */
float rudder_angle_kd   = 0.0f; /*< �ǶȻ�D���� */
float rudder_angle_limit = 30000.0f; /*< �ٶȻ������޷� */

/* �ٶȻ� */
float rudder_speed_kp = 205.0f; /*< ���ٶ�P���� */
float rudder_speed_ki = 0.0f; /*< ���ٶȻ�I����0.004 */
float rudder_speed_kd = 0.0f; /*< ���ٶȻ�D����0.05 */
float rudder_speed_limit = 30000.0f; /*< �ٶȻ������޷� */
float rudder_out_limit = 30000.0f; /*< ����޷���6020������30000*/

/* ��챵�� */
/* �ٶȻ� */
float chassis_speed_kp   = 10.5f; /*< �ٶȻ�P���� 18.0 */
float chassis_speed_ki   = 0.0f; /*< �ٶȻ�I����0.005 */
float chassis_speed_kd   = 0.00001f; /*<  �ٶȻ�D���� */
float chassis_speed_inc_lim = 16000.0f; /*< �ٶȻ������޷� */

/* ������ */
float chassis_current_kp = 2.5;//1.35; /*<  �ٶȻ�P���� */
float chassis_current_ki = 0.0f; /*<  �ٶȻ�I���� */
float chassis_current_kd = 0.0f; /*< �ٶȻ�D���� */
float chassis_current_inc_lim = 16000.0f; /*< �ٶȻ������޷� */

float chassis_out_lim = 16000.0f;
#endif
