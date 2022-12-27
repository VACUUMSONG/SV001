#include "Encoder.h"
#include "mylog.h"

//获取左电机速度
void Read_Left_Moto(){
    ModbusRTU MotorEncoder;
    motor_left_counter =  MotorEncoder.mb_Get_LeftMotor_RPA();
}

//获取右电机速度
void Read_Right_Moto(){
    ModbusRTU MotorEncoder;
    motor_right_counter = MotorEncoder.mb_Get_RightMotor_RPA();
}

Encoder::Encoder(int direction, MotorPosition left_or_right) {
    this->direction = direction;
    this->motor_position = left_or_right;
    
    if(motor_position == LeftMotor){
        // this->motorCountPin1  = MOTORL_COUNTPIN1;
        // this->motorCountPin2 = MOTORL_COUNTPIN2;
    }else{
        // this->motorCountPin1  = MOTORR_COUNTPIN1;
        // this->motorCountPin2 = MOTORR_COUNTPIN2;
    }
}

Encoder::~Encoder() {

}

void Encoder::init() {
    //编码器开启定时器
}

short Encoder::read() {
    //获取的编码器数据
    short count = 0;

    if (this->motor_position == LeftMotor){
        count = (short)motor_left_counter;
        // mylog("[left] counter:%d \n", (int) (count));
        //重置计数
        motor_left_counter =  0;
    }else if(this->motor_position == RightMotor){
        count = (short)motor_right_counter;
        // mylog("[right] counter:%d \n", (int) (count));
        //重置计数
        motor_right_counter = 0;
    }
    // mylog("right:%d, CurSpeed:%d", (int) (this->motor_position), (int)(i));

    return count*this->direction;
}
