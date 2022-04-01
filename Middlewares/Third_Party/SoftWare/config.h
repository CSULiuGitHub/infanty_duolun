#ifndef _CONFIG_H
#define _CONFIG_H
    #define RAD2DEG (57.296f) /* 1���� = 57.296�� */
    #define DEG8192 (22.75556f) /* 1�� ��Ӧ����Ƕȱ���ֵ��������6020��� */
    //#define NO_CAP

    /* �����˱�� ֻ�ܶ���һ��*/
    /* ��Դ��ֻ����5�� */
    //#define INFANTRY_3
    //#define INFANTRY_4
    //#define INFANTRY_5 /*< ��ͷ���� */
    //#define INFANTRY_5_DOUBLE /*< ˫ǹ�� */
    #define INFANTRY_6   /*< Ħ���ֵ��Ϊ3508*/
    //#define IMU
    //#define ADRC
    //#define DEBUG

    /* ң�������˿��� */
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

        #define MAX_PITCH_ANGLE_MOTOR (47000) //4685�� ��1269
        #define MIN_PITCH_ANGLE_MOTOR (38100) //3416�� ��4069  // (|motor_angle-3416|)/1269

        #define MAX_PITCH_ANGLE_IMU (4537) // 4704�� ��987
        #define MIN_PITCH_ANGLE_IMU (3550) // 3465�� ��4071 //(|imu_angle-4704|)/987 

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

        #define MAX_PITCH_ANGLE_MOTOR (71000) //4685�� ��1269       18000   1000    -4710  -17180
        #define MIN_PITCH_ANGLE_MOTOR (62000) //3416�� ��4069  // (|motor_angle-3416|)/1269   

        #define MAX_PITCH_ANGLE_IMU (16800) // 4704�� ��987         16800   1000
        #define MIN_PITCH_ANGLE_IMU (1000)  // 3465�� ��4071 //(|imu_angle-4704|)/987         

        #define UP_PITCH_ANGLE_MOTOR (62880)//62880
        #define UP_PITCH_ANGLE_IMU (32841)
        #define DOWN_PITCH_ANGLE_MOTOR (48220)
        #define DOWN_PITCH_ANGLE_IMU (47758)
    #endif


    #if defined(INFANTRY_5)
        #define ANGLE_BIAS -197

        #define LID_CLOSE_PULSE (2250)   /*< ���ָǹر�ֵ */
        #define TURN_ROUND_ANGLE (44000) /*< һ����ͷ��ƫ��ת��Ȧ��ֵ�� */

        #define MAX_PITCH_ANGLE_MOTOR (14700) /*< ��������������ֵ */
        #define MIN_PITCH_ANGLE_MOTOR (1300) 

        #define MAX_PITCH_ANGLE_IMU (45858) /*< ����IMU�������ֵ */
        #define MIN_PITCH_ANGLE_IMU (31852)

        #define UP_PITCH_ANGLE_MOTOR (15730)   /*< �������ϵĵ������ */
        #define UP_PITCH_ANGLE_IMU (33211)     /*< �������ϵ�IMU���� */
        #define DOWN_PITCH_ANGLE_MOTOR (1040)
        #define DOWN_PITCH_ANGLE_IMU (47508)
    #endif

    #ifndef LID_OPEN_PULSE
        #define LID_OPEN_PULSE (1000)
    #endif

#endif






