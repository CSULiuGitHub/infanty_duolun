#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  底盘控制相关参数
  * @attention 双闭环控制 
  *     外环（速度环）采用增量式PID控制器
  *     内环（电流环）采用增量式PID控制器
  *     调试方法：
  *         1.定义 #define CHASSIS_DEBUG，可通过debug在线修改参数
  *         2.定义 #define CHASSIS_WAVE，可输出底盘相关波形
  *         具体实现方法在chassis.c中的void Chassis_Init(void)函数中
  *
  *         每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  ****************************************************************************/

/* 进行调试前必须定义下面参数，其中波形参数只能定义一个 */
/* #define CHASSIS_DEBUG */
/* #define CHASSIS_WAVE */

#define CHASSIS_DEBUG
#define CHASSIS_WAVE

/* 舵向电机 */
/* 角度环 */
float rudder_angle_kp   = 0.25f; /*< 角度环P参数12 */
float rudder_angle_ki   = 0.0f; /*< 角度环I参数 */
float rudder_angle_kd   = 0.0f; /*< 角度环D参数 */
float rudder_angle_limit = 30000.0f; /*< 速度环增量限幅 */

/* 速度环 */
float rudder_speed_kp = 205.0f; /*< 角速度P参数 */
float rudder_speed_ki = 0.0f; /*< 角速度环I参数0.004 */
float rudder_speed_kd = 0.0f; /*< 角速度环D参数0.05 */
float rudder_speed_limit = 30000.0f; /*< 速度环增量限幅 */
float rudder_out_limit = 30000.0f; /*< 输出限幅，6020最大接收30000*/

/* 轮毂电机 */
/* 速度环 */
float chassis_speed_kp   = 10.5f; /*< 速度环P参数 18.0 */
float chassis_speed_ki   = 0.0f; /*< 速度环I参数0.005 */
float chassis_speed_kd   = 0.00001f; /*<  速度环D参数 */
float chassis_speed_inc_lim = 16000.0f; /*< 速度环增量限幅 */

/* 电流环 */
float chassis_current_kp = 2.5;//1.35; /*<  速度环P参数 */
float chassis_current_ki = 0.0f; /*<  速度环I参数 */
float chassis_current_kd = 0.0f; /*< 速度环D参数 */
float chassis_current_inc_lim = 16000.0f; /*< 速度环增量限幅 */

float chassis_out_lim = 16000.0f;
#endif
