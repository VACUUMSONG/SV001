#include "motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t MotorPin1, uint8_t MotorPin2,
             uint8_t MotorPWM, int direction) {

    this->MotorPin1 = MotorPin1;
    this->MotorPin2 = MotorPin2;
    this->MotorPWM = MotorPWM;
    this->direction = direction;
    
    //定义引脚
    pinMode(this->MotorPin1, INPUT);  //驱动芯片控制引脚
    pinMode(this->MotorPin2, INPUT);  //驱动芯片控制引脚
    pinMode(this->MotorPWM, OUTPUT);   //驱动芯片控制引脚，PWM调速
}

Motor::~Motor() {
}

void Motor::init() {
 
  
}

void Motor::forward(){
    pinMode(this->MotorPin1,OUTPUT);
    digitalWrite(this->MotorPin1, LOW);

    // digitalWrite(this->MotorPin2, HIGH);
    pinMode(this->MotorPin2,INPUT);
}

void Motor::backward(){

    // digitalWrite(this->MotorPin1, HIGH);
    pinMode(this->MotorPin1,INPUT);

    pinMode(this->MotorPin2,OUTPUT);
    digitalWrite(this->MotorPin2, LOW);
}

void Motor::stop(){
    ModbusRTU modbusRTU;
    modbusRTU.mb_Set_Motor_Stop();
}

void Motor::spin(int pwm) {

    //pwm是否在范围之内
    if (pwm > MAX_PWM) {
        pwm = MAX_PWM;
    } else if (pwm < MIN_PWM) {
        pwm = MIN_PWM;
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
