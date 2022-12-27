#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>
#include "Encoder.h"
#include "modbusRTU.h"

static volatile int16_t motor_left_counter;
static volatile int16_t motor_right_counter;

typedef enum{
    LeftMotor,RightMotor
} MotorPosition;

//编码器
//属性: 定时器  channel
//行为: 初始化  获取当前编码数据
class Encoder {
private:
    //方向调节系数
    int direction=1;

    // 左轮右轮
    MotorPosition motor_position;

public:
    Encoder(int direction, MotorPosition left_or_right);

    ~Encoder();

    //初始化
    void init();
    //读取编码器数据 
    short read();
};


#endif 
