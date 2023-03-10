#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define LED_PIN PA1

// 串口 BM1684
#define PIN_U1_RX      PA10
#define PIN_U1_TX      PA9
// 串口 Modbus
#define PIN_U5_RX      PD2
#define PIN_U5_TX      PC12

//最小超声避障距离 默认 30CM
#define MIN_DIST_AVOID_OBSTACLE  30

//最大和最小RPM
#define MAX_RPM         250
#define MIN_RPM         -250

//编码器获取数据的频率
#define MOVE_CTRL_RATE 100

// 轮子转一圈的信号数 4096线 编码电机 
#define WHEEL_TPR           (4096)

// 轮子的直径
#define WHEEL_DIAMETER      0.138f
// 左右轮子的距离
#define WHEEL_DISTANCE      0.384f

#endif 