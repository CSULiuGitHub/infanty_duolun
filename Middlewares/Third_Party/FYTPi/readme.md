# FYTPi软件库使用手册

[TOC]

## 修改记录

| 时间      | 版本 | 备注                       |
| --------- | ---- | -------------------------- |
| 2021-8-28 | v1.0 | 首次发布，仅供参考，未实测 |
|           |      |                            |
|           |      |                            |

## 简介

FYTPi是一个通用的嵌入式软件库，包含与嵌入式相关的数学公式、信号处理和自动控制等常见算法。

FYTPi目前包含如下模块

- fytpi_control（控制）：常见变种PID算法、ADRC算法
- fytpi_dsp（信号处理）：低通滤波、加权滤波
- fytpi_math（数学公式）：三角函数、平方根、限幅、斜坡
- fytpi_protocol（外设）：陀螺仪、电机、mini pc等通讯协议

## 使用方法

①将FYTPI文件夹放到工程文件中，并在keil mdk中设置文件包含路径

②包含fytpi.h头文件，定义需要使用的库为1，不使用为0

```C
/* define --------------------------------------------------------------------*/
#define USE_FYTPI_DSP      (1) /*< 使用DSP库 */
#define USE_FYTPI_CONTROL  (1) /*< 使用Control库 */
#define USE_FYTPI_MATH     (0) /*< 不使用Math库 */
#define USE_FYTPI_PROTOCOL (0) /*< 不使用protocol库 */
```

## 模块介绍

### fytpi_math

常用的数学函数库，包括基本的三角函数和平方根快速算法，以及限幅和斜坡等曲线。

- 快速三角函数：利用查表法减小计算量

  示例：

  ```C
  float fast_cos(int Angle);
  float fast_sin(int Angle);
  ```

  > 输入参数：整数角度，单位“度”
  > 返回值：查表结果，float类型

- 快速平方根：常见快速平方根算法

  示例：

  ```C
  float quake_sqrt(float f_Num);
  ```

  > 输入参数：float数
  > 返回值：平方根计算结果

- 绝对值：计算绝对值

  示例

  ```C
  float abs_float(float Num);
  int abs_int(int Num);
  int8_t abs_int8(int8_t Num);
  int16_t abs_int16(int16_t Num);
  int32_t abs_int32(int32_t Num);
  ```

  > 输入参数：目标数
  > 返回值：计算绝对值结果

- 限幅函数：限制目标数据的范围

  示例：

  ```C
  int constrain_int(int Num, int Low, int High);
  int8_t constrain_int8(int8_t Num, int8_t Low, int8_t High);
  int16_t constrain_int16(int16_t Num, int16_t Low, int16_t High);
  int32_t constrain_int32(int32_t Num, int32_t Low, int32_t High);
  uint8_t constrain_uint8(uint8_t Num, uint8_t Low, uint8_t High);
  uint16_t constrain_uint16(uint16_t Num, uint16_t Low, uint16_t High);
  uint32_t constrain_uint32(uint32_t Num, uint32_t Low, uint32_t High);
  float constrain_float(float Num, float Low, float High);
  ```

  > 输入参数：目标数，最小值，最大值
  > 返回值：限幅结果

- 斜坡函数：使目标值成斜坡变化

  示例：

  ```C
  float ramp_float(float Final, float Now, float Ramp);
  int ramp_int(int Final, int Now, int Ramp);
  int8_t ramp_int8(int8_t Final, int8_t Now, int8_t Ramp);
  int16_t ramp_int16(int16_t Final, int16_t Now, int16_t Ramp);
  int32_t ramp_int32(int32_t Final, int32_t Now, int32_t Ramp);
  uint8_t ramp_uint8(uint8_t Final, uint8_t Now, uint8_t Ramp);
  uint16_t ramp_uint16(uint16_t Final, uint16_t Now, uint16_t Ramp);
  uint32_t ramp_uint32(uint32_t Final, uint32_t Now, uint32_t Ramp);
  ```

  > 输入参数：最终值，当前值，斜率
  > 返回值：下一刻的值

### fytpi_control

常用的控制算法库，包括绝对式、增量式PID控制器、变结构PI控制器和ADRC等。

- 增量式PID

  示例：

  ```C
  /* 初始化 */
  void pid_init_increment(PID_INCREMENT_T *PID, float Kp, float Ki, float Kd,
      float IncLim);
  ```

  > 输入参数：PID结构体地址，P,I,D,增量限幅
  > 返回值：无

  ```C
  /* 计算输出 */
  float pid_increment_update(float Target, float Current, PID_INCREMENT_T *PID);
  ```

  > 输入参数：目标值，反馈值，PID结构体地址
  > 返回值：输出值

- 绝对式PID

  示例：

  ```C
  /* 初始化 */
  void pid_init_absolute(PID_ABSOLUTE_T *PID, float Kp, float Ki, float Kd,
      float Errlimit, float ErrPLim);
  ```

  > 输入参数：PID结构体地址，P,I,D,积分限幅，比例限幅
  > 返回值：无

  ```C
  /* 计算输出值 */
  float pid_absolute_update(float Target, float Current, PID_ABSOLUTE_T *PID);
  ```

  > 输入参数：目标值，实际值，PID结构体地址
  > 返回值：输出值

- 变结构PI控制器

  示例：

  ```C
  /* 改变PI值（绝对式） */
  void pi_tunning_absolute(float PMin, float PDynamic, float IMin, float IDynamic,
      float Kd, PID_ABSOLUTE_T* PID);
  ```

  >输入参数：p最小值，p变化范围，i最小值，i变化范围，d值，绝对式PID结构体
  >返回值：无

- ADRC自抗扰控制器

  示例：

### fytpi_dsp

常用的信号处理算法库，包括低通滤波、加权滤波等。

- 低通滤波

  示例：

  

- 加权滤波

  示例：

  ```C
  /* 四点加权滤波 */
  int rm4_int(int RawData, int RM4FilterBuf[4]);
  float rm4_float(float RawData, float RM4FilterBuf[4]);
  ```

  > 输入参数：原始数据，数据缓冲区
  > 返回值：滤波后的值

### fytpi_protocol

常用的通信协议算法库，包括minipc、陀螺仪和遥控器串口协议，电机CAN协议。

- minipc

  示例

- 陀螺仪

  示例

- 遥控器DR16

  示例：

  ```C
  /* 解析遥控器数据，并储存在RDMsg中 */
  void remote_data_msg_process(uint8_t *ReceiveDataBuf, REMOTE_DATA_T *RDMsg)
  ```

  > 输入参数：接收数据缓冲区，遥控数据结构体指针
  > 返回值：无

- 电机

  示例