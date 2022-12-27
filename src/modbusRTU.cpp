#include "modbusRTU.h"
#include <string.h>

// 中菱 电机驱动 Modbus协议
String data = "";  // 接收到的16进制字符串

// Modbus CRC
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
	unsigned char uchCRCHi = 0xFF; /* 高字节初始化值   */
	unsigned char uchCRCLo = 0xFF; /* 低字节初始化值   */
	unsigned uIndex;

	// CRC校验查表
	static unsigned char auchCRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40};

	static char auchCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40};
	while (usDataLen--)
	{
		uIndex = uchCRCLo ^ *puchMsg++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

ModbusRTU::ModbusRTU() {
}

ModbusRTU::~ModbusRTU(){
}

void ModbusRTU::init()
{
	HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);
	HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);

	//串口初始化
	Serial_Modbus.begin(115200);
	Serial_Modbus.println("UART Modbus Init...");
	delay(200);
	Serial_BM1684.begin(115200);
	Serial_BM1684.println("UART BM1684 Init...");
	delay(200);
}

// 不断执行
void ModbusRTU::spin(){
	
}

uint8_t ModbusRTU::toBCD(int val) {  
	// HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
	// Serial_BM1684.begin(115200);

	char resHex =  (char) (val%16 & 0x0F) | (val/16 << 4);  
	// Serial_BM1684.println(resHex,HEX);
	delay(200);
	
	return resHex;
}

void ModbusRTU::mb_Send_Cmd(uint8_t cmd[],uint8_t len)
{
	HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);
	Serial_Modbus.begin(115200);

	// 数据请求
	for (int i = 0; i < len; i++) {
		Serial_Modbus.write(cmd[i]); // write输出
	}
	delay(200); //等待读取完成
}

// 故障清除
void ModbusRTU::mb_Cls_Error() {
	uint8_t mb_clsError[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x06, 0x63, 0xCB};
	mb_Send_Cmd(mb_clsError,8);
}

// 电机急停
void ModbusRTU::mb_Set_Motor_EmerganceyStop()
{
	uint8_t mb_setEmergencyStop[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x05, 0x23, 0xCA};
	mb_Send_Cmd(mb_setEmergencyStop,8);
}

// 电机停机
void ModbusRTU::mb_Set_Motor_Stop(){
	uint8_t mb_setStop[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x07, 0xA2, 0x0B};
	mb_Send_Cmd(mb_setStop,8);
}

// 电机使能
void ModbusRTU::mb_Set_Motor_Enable() {
	
	// 设置速度模式
	uint8_t mb_setMode_Speed[8] = {0x01, 0x06, 0x20, 0x0D, 0x00, 0x03, 0x53, 0xC8};
	mb_Send_Cmd(mb_setMode_Speed,8);
	// S型加减速时间设定
	// TODO
	// 电机使能
	uint8_t mb_setStart[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0xE2, 0x0F};
	mb_Send_Cmd(mb_setStart,8);

	// // 左右电机同步转速设定100rpm
	// uint8_t mb_setSpeed_LR[13] = {0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, 0x00, 0x64, 0x00, 0x64, 0x23, 0x9C};
	// mb_Send_Cmd(mb_setSpeed_LR,13);
}

// 转速同步 
void ModbusRTU::mb_Set_Motor_Speed() {
	// 左右电机同步转速设定100rpm

	// 左转
	// uint8_t mb_setSpeed_LR[13] = {0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, 0x00, 0x64, 0x00, 0x64, 0x23, 0x9C};
	// 前进
	uint8_t mb_setSpeed_LR[13] = {0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, 0xFF, 0x9C, 0x00, 0x64, 0x92, 0x79};
	
	mb_Send_Cmd(mb_setSpeed_LR,13);
}

// 转速设定 -> 左电机
void ModbusRTU::mb_Set_Left_Moto(int rpm) {
	
	HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);
	Serial_Modbus.begin(115200);

	// rpm 限幅
    if (rpm > MAX_RPM) {
        rpm = MAX_RPM;
    } else if (rpm < MIN_RPM) {
        rpm = MIN_RPM;
    }

	// 写 左电机转速 0x88
	uint8_t mb_setSpeed_cmd[8]={};
	
	if(rpm>0){
		// 高八位 0x00
		uint8_t mb_setSpeed_LM[5] = {0x01, 0x06, 0x20, 0x88, 0x00};
		for(char i=0;i<5;i++){
			mb_setSpeed_cmd[i]=mb_setSpeed_LM[i];
		}

		int8_t rpmHex = toBCD(rpm);
		mb_setSpeed_cmd[5]=rpmHex;
	}else{
		// 高八位 0xFF
		uint8_t mb_setSpeed_LM[5] = {0x01, 0x06, 0x20, 0x88, 0xFF};
		for(char i=0;i<5;i++){
			mb_setSpeed_cmd[i]=mb_setSpeed_LM[i];
		}
		// 取补码
		int8_t rpmHex = toBCD((255-rpm));
		mb_setSpeed_cmd[5]=rpmHex;

	}

	unsigned short myCRC = CRC16(mb_setSpeed_cmd,6);
	mb_setSpeed_cmd[6]=(myCRC>>8)&0xFF;
	mb_setSpeed_cmd[7]=myCRC&0xFF;

	// Serial_BM1684.print(mb_setSpeed_cmd);

	// 数据请求
	for (int i = 0; i < 8; i++) {
		Serial_Modbus.write(mb_setSpeed_cmd[i]); // write输出
	}
	delay(200); //等待读取完成
}

// 转速设定 -> 右电机
void ModbusRTU::mb_Set_Right_Moto(int rpm) {
	// rpm 限幅
    if (rpm > MAX_RPM) {
        rpm = MAX_RPM;
    } else if (rpm < MIN_RPM) {
        rpm = MIN_RPM;
    }
	// 新数组
	uint8_t mb_setSpeed_cmd[8]={};

	// 写 右电机转速
	uint8_t mb_setSpeed_RM[5] = {0x01, 0x06, 0x20, 0x89, 0x00};
	for(char i=0;i<5;i++){
		mb_setSpeed_cmd[i]=mb_setSpeed_RM[i];
	}

	int8_t rpmHex = toBCD(rpm);
	mb_setSpeed_cmd[5]=rpmHex;

	unsigned short myCRC = CRC16(mb_setSpeed_cmd,6);
	mb_setSpeed_cmd[6]=(myCRC>>8)&0xFF;
	mb_setSpeed_cmd[7]=myCRC&0xFF;

	mb_Set_Motor_Enable();
	mb_Send_Cmd(mb_setSpeed_cmd,8);	
}

/*------------------*/
//  获取编码器速度
//  单位：RPM
/*------------------*/
int16_t ModbusRTU::mb_Get_LeftMotor_RPA() {

	HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);
	HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
	Serial_BM1684.begin(115200);
	Serial_Modbus.begin(115200);

	String data="";
	char mySpeedRpm[2]="";

	// 获取电机速度
	uint8_t mb_getSpeed_TwoMotor[8]={0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B};
	for (int i = 0; i < 8; i++) {                   // 读取电机转速
		Serial_Modbus.write(mb_getSpeed_TwoMotor[i]); // write输出
	}
	delay(200);

	// 数据接收
	while (Serial_Modbus.available())
	{ // 从串口中读取数据
		data += (char)Serial_Modbus.read(); // read读取
		Serial_Modbus.flush();
	}

	Serial_BM1684.println(data);
	delay(200);
	
	data.toCharArray(mySpeedRpm,5,3);
	Serial_BM1684.println(mySpeedRpm);
	delay(200);
	data = "";

	return 0;
}

int16_t ModbusRTU::mb_Get_RightMotor_RPA() {

	HardwareSerial Serial_Modbus(PIN_U5_RX, PIN_U5_TX);
	HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
	Serial_BM1684.begin(115200);
	Serial_Modbus.begin(115200);

	String data="";
	char mySpeedRpm[2]="";

	// 获取电机速度
	uint8_t mb_getSpeed_TwoMotor[8]={0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B};
	for (int i = 0; i < 8; i++) {                   // 读取电机转速
		Serial_Modbus.write(mb_getSpeed_TwoMotor[i]); // write输出
	}
	delay(200);

	// 数据接收
	while (Serial_Modbus.available())
	{ // 从串口中读取数据
		data += (char)Serial_Modbus.read(); // read读取
		Serial_Modbus.flush();
	}
	
	Serial_BM1684.print(data);
	delay(200);

	data.toCharArray(mySpeedRpm,3,2);

	Serial_BM1684.println(mySpeedRpm);
	delay(200);
	data = "";
	return 0;
}