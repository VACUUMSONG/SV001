#include "config.h"
#include "parser.h"
#include "modbusRTU.h"
#include "UsSensor.h"
#include "Car.h"
#include <STM32FreeRTOS.h>


/*==================== 任务创建 ========================*/
// LED 
TaskHandle_t blink;
// 命令读取
TaskHandle_t readCommandTask;
// 命令反馈
TaskHandle_t sendCommandTask;
// 超声波事件处理
TaskHandle_t ultrtasonicTask;

/*==================== 底盘驱动 ========================*/
ModbusRTU chassis;
/*==================== 小车相关 ==========================*/
// 小车线速度和角速度
float vel = 0, angular = 0;
// 通讯协议数据帧
uint8_t publishMsg[sBUFFER_SIZE];

// 左电机 编码器 轮子
Motor lmotor(1);
Encoder lencoder(-1, LeftMotor);
Wheel lwheel(lmotor, lencoder, true);

// 右电机 编码器 轮子
Motor rmotor(-1);
Encoder rencoder(1, RightMotor);
Wheel rwheel(rmotor,rencoder, false);

// SV001 底盘
Car sv001(lwheel,rwheel);

/*==================== LED =========================*/
static void vLEDFlashTask(void *pvParameters) {

  pinMode(LED_PIN, OUTPUT);

  // Flash led every 200 ms.
  for (;;) {
    // LED flash .
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
  }
}

/*==================== 数据反馈 =========================*/
void readCommand(void *para) { 
	//串口初始化
  HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
  Serial_BM1684.begin(115200);
  Serial_BM1684.print(" start to receive command.\n");
  
  enum frameState
  {
    State_Head1, State_Head2, State_Size, State_Type, 
    State_Velocity, State_Pid, State_Params,
    State_CheckSum, State_Handle
  };
  uint8_t frame_type; // velocity; pid; correction; 
  frameState state = State_Head1; // init with state==Head1
  uint8_t command[rBUFFER_SIZE]; // 指令

  //State machine
  // [head1 head2 size type data checksum ]
  // [0xAA  0x55  0x2D 0x01 ....  0xXX    ]
  for(;;){
    // Serial_BM1684.print("receive command...\n");
    // delay(1000);
    if (Serial_BM1684.available()==0){
      continue;
    }

    switch (state)
    {
      case State_Head1:             //waiting for frame header 1
          command[0] = Serial_BM1684.read();
          state = (command[0] == head1 ? State_Head2 : State_Head1);
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("recv head1 error : ->"<<(int)data[0]);
          }
          break;
      
      case State_Head2:             //waiting for frame header 2
          command[1] = Serial_BM1684.read();
          state = (command[1] == head2 ? State_Size : State_Head1);
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("recv head2 error : ->"<<(int)data[1]);
          }
          break;
          
      case State_Size:              //waiting for frame Size
          command[2] = Serial_BM1684.read();
          state = State_Type;
          break;
          
      case State_Type:              //waiting for data_type
          command[3] = Serial_BM1684.read();
          frame_type = command[3];
          if (frame_type == receiveType_velocity){
            state = State_Velocity;
          }else if (frame_type == receiveType_pid){
            state = State_Pid;
          }else if (frame_type == receiveType_params){
            state = State_Params;
          }else{
            state = State_Head1;
          }
          break;
      case State_Velocity:
          Serial_BM1684.readBytes(command+4, 6);
          state = State_CheckSum;
          break;
      case State_Pid: //TODO
          Serial_BM1684.readBytes(command+4, 6);
          state = State_CheckSum;
          break;
      case State_Params: // TODO
          Serial_BM1684.readBytes(command+4, 6);
          state = State_Head1;
          break;
      
      case State_CheckSum:         //waiting for frame CheckSum
          command[10] = Serial_BM1684.read();
          state = command[10] == checksum(command,10) ? State_Handle : State_Head1;
          if(state == State_Head1)
          {
              //ROS_INFO_STREAM ("check sum error! recv is  : ->"<<(int)data[frame_size -1]<<"  calc is "<<frame_sum);
          }
          break;
          
      case State_Handle:
          if (frame_type == receiveType_velocity){
            // parser to vel
            parse_rvelcommnad(command, vel, angular);
            //  update speed
            sv001.updateSpeed(vel, angular);
          }else if (frame_type == receiveType_pid){
            float kp,ki,kd; 
            parse_pid(command, kp, ki, kd);
            sv001.updatePid(kp, ki, kd);
            // TODO
          }else if (frame_type == receiveType_params){
            // TODO
            
          }else{
            // not should go here.
          }
          state = State_Head1;
          break;
      
      default:
          state = State_Head1;
          break;
    }
    Serial_BM1684.end();
  }
}

void sendCommand(void *pvParameters) {
  HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);
  Serial_BM1684.begin(115200);
  
  for(;;){
    // publish msg to master
    send_data msg;
    float linear_vel = sv001.getVel();
    float angular_vel = sv001.getAnguler();
    float pose_x = sv001.getPose_x();
    float pose_y = sv001.getPose_y();
    float pose_angular = sv001.getPose_angular();

    msg.x_v.fv = linear_vel;
    msg.y_v.fv = 0;
    msg.angular_v.fv = angular_vel;
    msg.x_pos.fv = pose_x;
    msg.y_pos.fv = pose_y;
    msg.pose_angular.fv = pose_angular;

    set_publishmsg(publishMsg, msg);
    Serial_BM1684.write(publishMsg, sBUFFER_SIZE);
  }
}

/*==================== 超声波处理 =========================*/
void taskUltrasonic(void *pvParameters){
  for(;;){
    // 超声波避障
    US_Avoid_Obstacle(); 
  }
}

// 初始化
void setup() {
  // 底盘初始化
  chassis.init();
  chassis.mb_Cls_Error();
  chassis.mb_Set_Motor_Enable();

  // SV001初始化
  sv001.init();

  // FreeRTOS 任务列表
  // 任务一：数据发送
  xTaskCreate(sendCommand, "sendCommandTask", 10000, NULL, 1, &sendCommandTask);
  // 任务二：数据解析
  xTaskCreate(readCommand,  "readCommandTask", 10000, NULL, 1, &readCommandTask);  
  // 任务三: 超声波避障
  // xTaskCreate(taskUltrasonic,"taskULtrasonic", 5000, NULL, 3, &ultrtasonicTask);
  // 任务四: LED灯显示
  xTaskCreate(vLEDFlashTask, "vLedFlashTask", configMINIMAL_STACK_SIZE+50, NULL, tskIDLE_PRIORITY+2, &blink);
    
  // 开启FreeRTOS 任务调度
  vTaskStartScheduler();
}

// 主循环
void loop() {
  sv001.spin(); 
}
