#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"
#include "ModbusRTU.h"
#include <math.h>

//马达
//属性:定时器  channel 第一个GPIO 第一个DPIO编号  第一个GPIO 第一个DPIO编号
//行为:马达转动
class Motor {
private:
    uint8_t MotorPin1;  //驱动芯片控制引脚
    uint8_t MotorPin2;  //驱动芯片控制引脚
    uint8_t MotorPWM;  //驱动芯片控制引脚，PWM调速

    //方向校正系数 1: -1:
    int direction = 1;
public:
    Motor(uint8_t MotorPin1, uint8_t MotorPin2,
             uint8_t MotorPWM, int direction);

    ~Motor();
    //初始化方法
    void init();

    void forward();
    void backward();
    void stop();

    /**
     * 马达转动 前  后
     * @param pwm 正 前  负 后
     */
    void spin(int pwm);
};


#endif //MOTOR_H
