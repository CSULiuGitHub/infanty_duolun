#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  ��̨������ز��� ��С�糵��ز���
  * @attention ˫�ջ����� 
  *     �⻷���ǶȻ������ñ�ṹPI����������С�������ӿ���Ӧ
  *     �ڻ����ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define HOLDER_DEBUG����ͨ��debug�����޸Ĳ���
  *         2.���� #define HOLDER_PITCH_WAVE�������PITCH�Შ��
  *         3.���� #define HOLDER_YAW_WAVE, �����YAW�Შ��
  *         4.���� #define HOLDER_VISION_WAVE, ������Ӿ�����
  *         ����ʵ�ַ�����holder.c�е�static void Holder_Pid_Manual(void)������
  *
  *         ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
/* #define HOLDER_DEBUG */
/* #define HOLDER_PITCH_WAVE */
/* #define HOLDER_YAW_WAVE */
/* #define HOLDER_VISION_WAVE */

//#define HOLDER_VISION_WAVE
#define HOLDER_DEBUG
#define HOLDER_PITCH_WAVE
//#define VISION_NO_CP
#ifdef DEBUG
//    #define HOLDER_DEBUG
//    #define HOLDER_YAW_WAVE
//    #define HOLDER_PITCH_WAVE
    //#define HOLDER_VISION_WAVE
#endif

/*****************************���л���ͨ�ò���**********************************/
float rc_pitch_bias = 0.08f; /*< ң����PitchĿ��Ƕ���С����0.08 */
float rc_yaw_bias = -0.18f;  /*< ң����YawĿ��Ƕ���С���� */
float key_pitch_bias = 3.0f; /*< ���PitchĿ��Ƕ���С���� */
float key_yaw_bias = 1.0f;   /*< ���PitchĿ��Ƕ���С���� */
int32_t key_yaw_limit = 250; /*< ���YAW�ٶ��޷� */

/*****************************��ͬ������ͬ����**********************************/
#ifdef INFANTRY_3 /*< ���Ų���*/
/* YAW�� */
float yaw_angle_tunning_kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_min_kp     = 0.10f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_angle_vision_tunning_kp = 0.08f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_vision_min_kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_vision_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_vision_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_vision_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_speed_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_speed_vision_kp = 5.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_vision_kd = 0.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_angle_lpf_bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float yaw_speed_lpf_bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float yaw_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_ki_limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float yaw_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float yaw_speed_ki_limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float yaw_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float yaw_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float pitch_angle_tunning_kp = 0.085f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float pitch_angle_min_kp     = 0.0095f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float pitch_angle_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_angle_vision_tunning_kp = 0.002f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float pitch_angle_vision_min_kp     = 0.00035f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float pitch_angle_vision_tunning_ki = 0.05f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_vision_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_vision_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_speed_kp = 300.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_ki = 0.8f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float pitch_speed_vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_vision_ki = 0.5f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float pitch_angle_lpf_bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float pitch_speed_lpf_bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float pitch_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float pitch_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float pitch_speed_ki_limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float pitch_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float pitch_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_3*/

#ifdef INFANTRY_4
/* YAW�� */
float yaw_angle_tunning_kp = 0.065f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_min_kp     = 0.02f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_kd         = 0.01f;  /*< �ǶȻ�D������΢�֣� */

float yaw_angle_vision_tunning_kp = 0.02f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_vision_min_kp     = 0.02f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_vision_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_vision_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_vision_kd         = 0.01f;  /*< �ǶȻ�D������΢�֣� */

float yaw_speed_kp = 9.5f;           /*< �ٶȻ�����P�������� */
float yaw_speed_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_kd = 8.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_speed_vision_kp = 10.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_vision_kd = 8.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_angle_lpf_bias = 0.7f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float yaw_speed_lpf_bias = 0.7f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float yaw_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float yaw_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float yaw_speed_ki_limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float yaw_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float yaw_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float pitch_angle_tunning_kp = 0.008f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float pitch_angle_min_kp     = 0.0055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float pitch_angle_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_angle_vision_tunning_kp = 0.002f; /*< �ǶȻ�����P�仯ֵ����ṹ������0.001f */ 
float pitch_angle_vision_min_kp     = 0.00055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������0.00035f */
float pitch_angle_vision_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_vision_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_vision_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_speed_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float pitch_speed_vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */


float pitch_angle_lpf_bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float pitch_speed_lpf_bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float pitch_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float pitch_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float pitch_speed_ki_limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float pitch_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float pitch_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_4*/

#ifdef INFANTRY_5
/* YAW�� ƫ��*/
float yaw_angle_tunning_kp = 0.04f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_min_kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_angle_vision_tunning_kp = 0.03f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_vision_min_kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_vision_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_vision_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_vision_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_speed_kp = 40.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_ki = 0.001f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_kd = 1.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_speed_vision_kp = 10.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_vision_kd = 5.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_angle_lpf_bias = 0.7f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float yaw_speed_lpf_bias = 0.7f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float yaw_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_ki_limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float yaw_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float yaw_speed_ki_limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float yaw_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float yaw_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� ���� */
float pitch_angle_tunning_kp = 0.008f; /*< �ǶȻ�����P�仯ֵ����ṹ������0.008 */
float pitch_angle_min_kp     = 0.0055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������0.0055 */
float pitch_angle_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_angle_vision_tunning_kp = 0.002f; /*< �ǶȻ�����P�仯ֵ����ṹ������0.001f */ 
float pitch_angle_vision_min_kp     = 0.00055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������0.00035f */
float pitch_angle_vision_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_vision_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_vision_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_speed_kp = 450.0f;           /*< �ٶȻ�����P�������� ��120 */
float pitch_speed_ki = 0.0f;            /*< �ٶȻ�����I�����֣�   0.14 */
float pitch_speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣�    3.0*/

float pitch_speed_vision_kp = 250.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_vision_ki = 0.0f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */


float pitch_angle_lpf_bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float pitch_speed_lpf_bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float pitch_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float pitch_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float pitch_speed_ki_limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float pitch_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float pitch_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */


/********************************��С�糵 �ǶȻ������ص����ٶȻ�����***************************/
/* YAW */
float Pinwheel_Yaw_Angle_Tunning_Kp = 1.8f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Min_Kp     = 0.0f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Yaw_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Min_Ki     = 0.001f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Yaw_Angle_Kd         = 0.8f;    /*< �ǶȻ�D������΢�֣� */

/* PITCH */
float Pinwheel_Pitch_Angle_Tunning_Kp = 0.08f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Min_Kp     = 0.04f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float Pinwheel_Pitch_Angle_Tunning_Ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Min_Ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float Pinwheel_Pitch_Angle_Kd         = 0.02f;    /*< �ǶȻ�D������΢�֣� */
#endif /*< ifdef INFANTRY_5*/

#ifdef INFANTRY_6
/* YAW�� */
float yaw_angle_tunning_kp = 0.12f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_min_kp     = 0.006f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_angle_vision_tunning_kp = 0.05f;  /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float yaw_angle_vision_min_kp     = 0.03f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float yaw_angle_vision_tunning_ki = 0.0f;  /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float yaw_angle_vision_min_ki     = 0.0f;  /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float yaw_angle_vision_kd         = 0.0f;  /*< �ǶȻ�D������΢�֣� */

float yaw_speed_kp = 35.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_ki = 0.0f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_kd = 18.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_speed_vision_kp = 30.0f;           /*< �ٶȻ�����P�������� */
float yaw_speed_vision_ki = 0.01f;         /*< �ٶȻ�����I�����֣� */
float yaw_speed_vision_kd = 15.0f;           /*< �ٶȻ�����D��΢�֣� */

float yaw_angle_lpf_bias = 0.9f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float yaw_speed_lpf_bias = 0.9f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float yaw_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_ki_limit = 999.0f;  /*< �ǶȻ�I��������޷� */
float yaw_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float yaw_speed_ki_limit = 10000.0f;  /*< �ٶȻ�I��������޷� */

float yaw_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float yaw_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float yaw_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */

/* PITCH�� */
float pitch_angle_tunning_kp = 0.009f; /*< �ǶȻ�����P�仯ֵ����ṹ������ */
float pitch_angle_min_kp     = 0.029f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������ */
float pitch_angle_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_kd         = 0.005f;    /*< �ǶȻ�D������΢�֣� */

float pitch_angle_vision_tunning_kp = 0.002f; /*< �ǶȻ�����P�仯ֵ����ṹ������0.001f */ 
float pitch_angle_vision_min_kp     = 0.00055f; /*< �ǶȻ�����P��С�̶�ֵ����ṹ������0.00035f */
float pitch_angle_vision_tunning_ki = 0.0f;    /*< �ǶȻ�����I�仯ֵ����ṹ���֣� */
float pitch_angle_vision_min_ki     = 0.0f;    /*< �ǶȻ�����I��С�̶�ֵ����ṹ���֣� */
float pitch_angle_vision_kd         = 0.0f;    /*< �ǶȻ�D������΢�֣� */

float pitch_speed_kp = 270.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_ki = 0.0f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float pitch_speed_vision_kp = 325.0f;           /*< �ٶȻ�����P�������� */
float pitch_speed_vision_ki = 0.12f;            /*< �ٶȻ�����I�����֣� */
float pitch_speed_vision_kd = 0.0f;            /*< �ٶȻ�����D��΢�֣� */

float pitch_angle_lpf_bias = 0.8f;    /*< �ǶȻ���ͨ�˲�ϵ�� */
float pitch_speed_lpf_bias = 0.8f;    /*< �ٶȻ���ͨ�˲�ϵ�� */

float pitch_angle_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */
float pitch_speed_kp_limit = 99999.0f;  /*< �ٶȻ�P��������޷� */
float pitch_speed_ki_limit = 9999.0f;  /*< �ٶȻ�I��������޷� */

float pitch_angle_vision_kp_limit = 99999.0f;  /*< �ǶȻ�P��������޷� */
float pitch_angle_vision_ki_limit = 99999.0f;  /*< �ǶȻ�I��������޷� */

float pitch_output_limit   = 28000.0f;  /*< ����޷���6020������30000 */

#endif /*< ifdef INFANTRY_6*/

#endif /*< ifdef HOLDER_CONFIG_H*/
