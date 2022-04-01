#ifndef _CONFIG_H
#define _CONFIG_H
    #define RAD2DEG (57.296f) /* 1弧度 = 57.296度 */
    #define DEG8192 (22.75556f) /* 1度 对应电机角度编码值，适用于6020电机 */
    //#define NO_CAP

    /* 机器人编号 只能定义一个*/
    /* 本源码只适用5号 */
    //#define INFANTRY_3
    //#define INFANTRY_4
    //#define INFANTRY_5 /*< 换头步兵 */
    //#define INFANTRY_5_DOUBLE /*< 双枪管 */
    #define INFANTRY_6   /*< 摩擦轮电机为3508*/
    //#define IMU
    //#define ADRC
    //#define DEBUG

    /* 遥控器拨杆开关 */
    #define SUP   (1)
    #define SMID  (3)
    #define SDOWN (2)
    
    #define FRONT      (0)
    #define RIGHT      (1)
    #define CLOCK_WISE (2)

    #if defined(INFANTRY_3)
        #define ANGLE_BIAS 150

        #define LID_CLOSE_PULSE (2450)
        #define TURN_ROUND_ANGLE (83000)

        #define MAX_PITCH_ANGLE_MOTOR (47000) //4685上 差1269
        #define MIN_PITCH_ANGLE_MOTOR (38100) //3416下 中4069  // (|motor_angle-3416|)/1269

        #define MAX_PITCH_ANGLE_IMU (4537) // 4704下 差987
        #define MIN_PITCH_ANGLE_IMU (3550) // 3465上 中4071 //(|imu_angle-4704|)/987 

        #define UP_PITCH_ANGLE_MOTOR (47220)
        #define UP_PITCH_ANGLE_IMU (34121)
        #define DOWN_PITCH_ANGLE_MOTOR (34270)
        #define DOWN_PITCH_ANGLE_IMU (46908)
    #endif

    #if defined(INFANTRY_4)
        #define ANGLE_BIAS -103
        #define LID_CLOSE_PULSE (2000)
        #define TURN_ROUND_ANGLE (78000)

        #define MAX_PITCH_ANGLE_MOTOR (49000)
        #define MIN_PITCH_ANGLE_MOTOR (38300)

        #define MAX_PITCH_ANGLE_IMU (4537)
        #define MIN_PITCH_ANGLE_IMU (3550)

        #define UP_PITCH_ANGLE_MOTOR (48090)
        #define UP_PITCH_ANGLE_IMU (34251)
        #define DOWN_PITCH_ANGLE_MOTOR (36080)
        #define DOWN_PITCH_ANGLE_IMU (45468)
    #endif


    #if defined(INFANTRY_5_DOUBLE)
        #define LID_CLOSE_PULSE (1000)
        #define LID_OPEN_PULSE (2250)
    #endif

    #if defined(INFANTRY_6)
        #define ANGLE_BIAS -32.0f

        #ifndef LID_CLOSE_PULSE
        #define LID_CLOSE_PULSE (2850)
        #endif

        #define TURN_ROUND_ANGLE (88000)

        #define MAX_PITCH_ANGLE_MOTOR (71000) //4685上 差1269       18000   1000    -4710  -17180
        #define MIN_PITCH_ANGLE_MOTOR (62000) //3416下 中4069  // (|motor_angle-3416|)/1269   

        #define MAX_PITCH_ANGLE_IMU (16800) // 4704下 差987         16800   1000
        #define MIN_PITCH_ANGLE_IMU (1000)  // 3465上 中4071 //(|imu_angle-4704|)/987         

        #define UP_PITCH_ANGLE_MOTOR (62880)//62880
        #define UP_PITCH_ANGLE_IMU (32841)
        #define DOWN_PITCH_ANGLE_MOTOR (48220)
        #define DOWN_PITCH_ANGLE_IMU (47758)
    #endif


    #if defined(INFANTRY_5)
        #define ANGLE_BIAS -197

        #define LID_CLOSE_PULSE (2250)   /*< 弹仓盖关闭值 */
        #define TURN_ROUND_ANGLE (44000) /*< 一键掉头（偏航转半圈的值） */

        #define MAX_PITCH_ANGLE_MOTOR (14700) /*< 俯仰电机反馈最大值 */
        #define MIN_PITCH_ANGLE_MOTOR (1300) 

        #define MAX_PITCH_ANGLE_IMU (45858) /*< 俯仰IMU反馈最大值 */
        #define MIN_PITCH_ANGLE_IMU (31852)

        #define UP_PITCH_ANGLE_MOTOR (15730)   /*< 俯仰向上的电机反馈 */
        #define UP_PITCH_ANGLE_IMU (33211)     /*< 俯仰向上的IMU反馈 */
        #define DOWN_PITCH_ANGLE_MOTOR (1040)
        #define DOWN_PITCH_ANGLE_IMU (47508)
    #endif

    #ifndef LID_OPEN_PULSE
        #define LID_OPEN_PULSE (1000)
    #endif

#endif






