#include "motor.h"
#include <Arduino.h>

Motor::Motor(int direction) {
    this->direction = direction;
}

Motor::~Motor() {
}

void Motor::init() {
}

void Motor::forward(){
}

void Motor::backward(){
}

void Motor::stop(){
    ModbusRTU modbusRTU;
    modbusRTU.mb_Set_Motor_Stop();
}

void Motor::spin(int pwm) {

    //pwm是否在范围之内
    if (pwm > MAX_RPM) {
        pwm = MAX_RPM;
    } else if (pwm < MIN_RPM) {
        pwm = MIN_RPM;
    }
    
    //乘以校正系数
    pwm *= this->direction;
    
    //判断方向
    if (pwm > 0) {
        //前
        this->forward();
    } else if (pwm < 0) {
        //后
        this->backward();
    } else {
        //停下来
        this->stop();
    }
    
    //速度pwm
    ModbusRTU modbusRTU;
    // modbusRTU.mb_Set_Motor_Speed(this->MotorPWM,abs(pwm));
}
