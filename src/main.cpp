#include "config.h"
#include "modbusRTU.h"
// #include <Ultrasonic.h>

ModbusRTU modbusRTU;

const int ledPin = PC6;

void setup()
{
  // 串口初始化
  modbusRTU.init();
  modbusRTU.mb_Cls_Error();
  modbusRTU.mb_Set_Motor_Enable();

  // 速度同步100
  // modbusRTU.mb_Set_Left_Moto(100);
  // modbusRTU.mb_Set_Right_Moto(-100);
  modbusRTU.mb_Set_Motor_Speed(); 
  
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
  
  modbusRTU.mb_Get_LeftMotor_RPA();
  modbusRTU.mb_Get_RightMotor_RPA();

  delay(10000);
  modbusRTU.mb_Set_Motor_Stop();
}
