#include "config.h"
#include <HardwareSerial.h>
// #include "modbusRTU.h"
// #include <Ultrasonic.h>

HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);

// 获取电机速度
uint8_t mb_getSpeed_TwoMotor[8] = {0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B};
// 清除故障
uint8_t mb_clsError[8] ={0x01, 0x06, 0x20, 0x0E, 0x00, 0x06, 0x63, 0xCB};

// 设置速度模式
uint8_t mb_setMode_Speed[8] = {0x01, 0x06, 0x20, 0x0D, 0x00, 0x03, 0x53, 0xc8};
// 电机使能
uint8_t mb_setMode_Enable[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0xE2, 0x0F};
// 同步转速1000rpm
uint8_t mb_setSpeed_100[13] = {0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, 0x00, 0x64, 0x00, 0x64, 0x23, 0x9C};

uint8_t mb_setStop[8] ={0x01, 0x06, 0x20, 0x0E, 0x00, 0x07, 0xA2, 0x0B};


const int ledPin = PC6;
String resData = "";


void setup()
{
  // 串口初始化
  Serial_BM1684.begin(115200);
  Serial_BM1684.println("BM1684 Init...");

  Serial_Modbus.begin(115200);
  Serial_Modbus.println("Modbus Init...");

  for (int i = 0; i < 8; i++) {                   // 清除故障
    Serial_Modbus.write(mb_clsError[i]);
  }
  delay(200);
  for (int i = 0; i < 8; i++) {                   // 速度模式
    Serial_Modbus.write(mb_setMode_Speed[i]);
  }
  delay(200);
  for (int i = 0; i < 8; i++) {                   // 使能
    Serial_Modbus.write(mb_setMode_Enable[i]);
  }
  delay(200);
  for (int i = 0; i < 13; i++) {                   // 同步100 
    Serial_Modbus.write(mb_setSpeed_100[i]);
  }
  delay(100);

  // GPIO初始化
  pinMode(ledPin, OUTPUT);
}

void loop()
{

  // for (int i = 0; i < 8; i++) {                   // 读取电机转速
  //   Serial_Modbus.write(mb_getSpeed_TwoMotor[i]); // write输出
  // }

  // //数据接收
  // while (Serial_Modbus.available() ) {// 从串口中读取数据
  //   resData += (char)Serial_Modbus.read();  // read读取
  //   Serial_Modbus.flush(); //等待，将之前的串口操作指令发送完毕
  // }

  // Serial_BM1684.print(resData);

  // resData = "";

  // LED
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(ledPin, LOW);
  delay(2000);

  for (int i = 0; i < 8; i++) {         // 读取电机转速
    Serial_Modbus.write(mb_setStop[i]); // write输出
  }
  delay(100);
}
