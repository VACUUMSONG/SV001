#include "UsSensor.h"
#include "config.h"

HardwareSerial Serial_BM1684(PIN_U1_RX, PIN_U1_TX);

void setup()
{
    Serial_BM1684.begin(115200);
    Serial_BM1684.print(" start Ultrasonic .\n");
}

void loop()
{
    Serial_BM1684.print("FL Sensor:");
    Serial_BM1684.println(US_get_FL_Dist());
}