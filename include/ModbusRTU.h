#ifndef MODBUSRTU_H
#define MODBUSRTU_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

class ModbusRTU{
private:
    // uint8_t SerialPinRx;
    // uint8_t SerialPinTx;

public:
    ModbusRTU();
    ~ModbusRTU();
    // 初始化
    void init();
    // 不断执行
    void spin();

    // 进制转化
    uint8_t toHex(int val);
    // 指令发送
    void mb_Send_Cmd(uint8_t cmd[],uint8_t len);
    // 速度反馈 
    void mb_Speed_callBack();

    // 电机_常用功能
    void mb_Cls_Error();
    void mb_Set_Motor_Enable();
    void mb_Set_Motor_Stop();
    void mb_Set_Motor_EmerganceyStop();

    // 前进
    void mb_Set_Motor_Forward();
    // 后退
    void mb_Set_Motor_Backward();
    // 左转
    void mb_Set_Motor_Left();
    // 右转
    void mb_Set_Motor_Right();

    // 电机_速度设定
    void mb_Set_Motor_Speed();
    void mb_Set_Left_Moto(int rpm);
    void mb_Set_Right_Moto(int rpm);

    // 电机_速度读取
    int16_t mb_Get_LeftMotor_RPA();
    int16_t mb_Get_RightMotor_RPA();

};

#endif