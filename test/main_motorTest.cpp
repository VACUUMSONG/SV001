#include "config.h"
#include "modbusRTU.h"
// #include <Ultrasonic.h>
#include <STM32FreeRTOS.h>

ModbusRTU modbusRTU;

const int ledPin = PC6;

void setup()
{
  // 串口初始化
  modbusRTU.init();
  modbusRTU.mb_Cls_Error();
  modbusRTU.mb_Set_Motor_Enable();
  
  // GPIO初始化
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  // LED
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
  
  // 速度设定 前进
  modbusRTU.mb_Set_Left_Moto(10);
  modbusRTU.mb_Set_Right_Moto(-10);
  // 速度读取
  modbusRTU.mb_Get_LeftMotor_RPA();
  modbusRTU.mb_Get_RightMotor_RPA();
  delay(5000);
  modbusRTU.mb_Set_Left_Moto(30);
  modbusRTU.mb_Set_Right_Moto(-30);
  // 速度读取
  modbusRTU.mb_Get_LeftMotor_RPA();
  modbusRTU.mb_Get_RightMotor_RPA();
  delay(5000);
  modbusRTU.mb_Set_Left_Moto(60);
  modbusRTU.mb_Set_Right_Moto(-60);
  // 速度读取
  modbusRTU.mb_Get_LeftMotor_RPA();
  modbusRTU.mb_Get_RightMotor_RPA();
  delay(5000);
  modbusRTU.mb_Set_Motor_Stop();
}