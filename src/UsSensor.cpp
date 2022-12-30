#include <UsSensor.h>
#include "config.h"
#include "ModbusRTU.h"

// 前侧三个超声波
Ultrasonic Front_Left( FL_PIN_TRIG, FL_PIN_ECHO);	
Ultrasonic Front_Mid ( FM_PIN_TRIG, FM_PIN_ECHO);
Ultrasonic Front_Right(FR_PIN_TRIG, FR_PIN_ECHO);
// 后侧两个超声波
Ultrasonic Back_Left ( BL_PIN_TRIG, BL_PIN_ECHO);	
Ultrasonic Back_Right( BR_PIN_TRIG, BR_PIN_ECHO);

void UsSensor_init(){

}

float US_get_FL_Dist(){
    return Front_Left.read(CM);
}

float US_get_FM_Dist(){
    return Front_Mid.read(CM);
}

float US_get_FR_Dist(){
    return Front_Right.read(CM);
}

float US_get_BL_Dist(){
    return Back_Left.read(CM);
}

float US_get_BR_Dist(){
    return  Back_Right.read(CM);
}

// 超声波避障
void US_Avoid_Obstacle(){
    // 小车检测到的距离
    float dist_FL,dist_FM,dist_FR,dist_BL,dist_BR;
    
    dist_FL = US_get_FL_Dist();
    dist_FM = US_get_FM_Dist();
    dist_FR = US_get_FR_Dist();
    dist_BL = US_get_BL_Dist();
    dist_BR = US_get_BR_Dist();

    ModbusRTU car;

    // 前路障碍
    if(dist_FM<MIN_DIST_AVOID_OBSTACLE || dist_FL<MIN_DIST_AVOID_OBSTACLE || dist_FR <MIN_DIST_AVOID_OBSTACLE){
        //方向选择
        if(dist_FL > dist_FR){ //左转
            car.mb_Set_Motor_Left();
        }
        if(dist_FR > dist_FL){ //右转
            car.mb_Set_Motor_Right();
        }
    }

    // 后路障碍
    if(dist_BL<MIN_DIST_AVOID_OBSTACLE || dist_BR < MIN_DIST_AVOID_OBSTACLE){
        //方向选择
        if(dist_BL > dist_BR){ //右转
            car.mb_Set_Motor_Right();
        }
        if(dist_BR > dist_BL){ //左转
            car.mb_Set_Motor_Left();
        }
    }
    
}