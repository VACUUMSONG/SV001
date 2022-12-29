#include "Encoder.h"
#include "mylog.h"

//获取左电机速度
int16_t Read_Left_Moto(){
    ModbusRTU MotorEncoder;
    motor_left_counter =  MotorEncoder.mb_Get_LeftMotor_RPA();
    return motor_left_counter;
}

//获取右电机速度
int16_t Read_Right_Moto(){
    ModbusRTU MotorEncoder;
    motor_right_counter = MotorEncoder.mb_Get_RightMotor_RPA();
    return motor_right_counter;
}

Encoder::Encoder(int direction, MotorPosition left_or_right) {
    this->direction = direction;
    this->motor_position = left_or_right;
    
    if(motor_position == LeftMotor){
    }else{
    }
}

Encoder::~Encoder() {

}

void Encoder::init() {
    //编码器开启定时器
    if(this->motor_position ==LeftMotor){
        Read_Left_Moto();
    }else if(this ->motor_position ==RightMotor){
        Read_Right_Moto();
    }

}

short Encoder::read() {
    //获取的编码器数据
    short count = 0;

    if (this->motor_position == LeftMotor){
        count = (short)Read_Left_Moto();
        // mylog("[left] counter:%d \n", (int) (count));
        //重置计数
        motor_left_counter =  0;
    }else if(this->motor_position == RightMotor){
        count = (short)Read_Right_Moto();
        // mylog("[right] counter:%d \n", (int) (count));
        //重置计数
        motor_right_counter = 0;
    }
    // mylog("right:%d, CurSpeed:%d", (int) (this->motor_position), (int)(i));

    return count*this->direction;
}
