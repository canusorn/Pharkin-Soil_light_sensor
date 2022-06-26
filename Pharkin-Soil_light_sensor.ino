/*

  blynk virtual pin

  V0 - moisture ความชื้น
  V1 - light แสง
  V2 - relay รีเลย์ปั้ม
  V3 - cw มอเตอร์หมุนตามเข็ม
  V4 - ccw มอเตอร์หมุนทวนเข็ม


  light sensor:
  VCC  <-> 3V3
  GND  <-> GND
  SDA  <-> D2
  SCL  <-> D1

  soil sensor
  VCC  <-> 5V
  GND  <-> GND
  A0   <-> A0
*/

// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLBPycYavo" // จากหน้า blynk template
#define BLYNK_DEVICE_NAME "soil light"   // จากหน้า blynk template

#define BLYNK_FIRMWARE_VERSION "0.0.1"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// ตั้งค่าช่วง แสง
#define MIN_LUX 10000
#define MAX_LUX 13000
// ตั้งค่าช่วง ความชื้น
#define MIN_MOISTURE 50
#define MAX_MOISTURE 69
// คาลิเบทความชื้น
#define ANALOG_WATER 500    // ค่าเมื่ออยู่ในน้ำ
#define ANALOG_NOWATER 1024 // ค่าเมื่อไม่อยู่ในน้ำ
// ขาต่อใช้งาน
#define MOTOR1 D3 // ขามอเตอร์ IN1 IN3
#define MOTOR2 D4 // ขามอเตอร์ IN2 IN4
#define RELAY D5  // ขาต่อรีเลย์คุมปั้มน้ำ

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h" //blynk ติดตังจาก library manager
#include <BH1750FVI.h>   // BH1750FVI ของ PeterEmbedded ติดตังจาก library manager

// Create the Lightsensor instance
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

unsigned long previousMillis = 0;
uint16_t lux, moisture;
bool relayOn = false; // flag relay on
uint8_t cw, ccw;      // timer to control motor

/*
0
*/
uint8_t light_state;

void setup()
{
  Serial.begin(115200);
  LightSensor.begin();

  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);
  digitalWrite(RELAY, LOW);
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(RELAY, OUTPUT);

  delay(100);

  BlynkEdgent.begin();
}

void loop()
{
  BlynkEdgent.run();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000)
  { //ทำงานทุก 1 วินาที
    previousMillis = currentMillis;

    lux = LightSensor.GetLightIntensity();
    Serial.print("Light: " + String(lux));

    moisture = map(analogRead(A0), 1024, 500, 0, 100);
    Serial.println("\tSoli Moisture : " + (String)moisture);

    // update blynk
    Blynk.virtualWrite(V0, moisture);
    Blynk.virtualWrite(V1, lux);

    // เงื่อนไขจากความชื้น
    if (moisture <= MIN_MOISTURE)
    {
      digitalWrite(RELAY, HIGH);
      relayOn = true;
    }
    else if (moisture > MAX_MOISTURE)
    {
      digitalWrite(RELAY, LOW);
      relayOn = false;
    }
    Blynk.virtualWrite(V2, relayOn);

    // เงื่อนไขจากแสง
    if (light_state == 0 && lux > MAX_LUX && cw == 0 && ccw == 0) //ตรวจจับได้ว่าค่าความเข้มแสงมีค่าเกินกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ตัวหนึ่งหมุนตามเข็มนาฬิกาและมอเตอร์อีกตัวหนึ่งหมุนทวนเข็มนาฬิกาเป็นเวลา 20 วินาที
    {
      light_state = 1;
      cw = 20;
    }
    else if (light_state == 1 && lux < MIN_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงตรวจจับได้ว่าค่าความเข้มแสงมีค่าต่ำกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ทั้งสอง 2 ตัวหมุนต่อในทิศทางเดิมจากตำแหน่งเดิมเป็นเวลาอีก 20 วินาที
    {
      light_state = 2;
      cw = 20;
    }
    else if (light_state == 2 && lux > MAX_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงสามารถตรวจจับได้ว่าค่าความเข้มแสงนั้นมีค่ามากกว่าช่วงที่กำหนดอีกครั้งให้ทำการสั่งการให้มอเตอร์หมุนทวนเข็มนาฬิกาเป็นเวลา 30 วินาที
    {
      light_state = 3;
      ccw = 30;
    }
    else if (light_state == 3 && lux < MIN_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงตรวจจับได้ว่าค่าความเข้มแสงมีค่าต่ำกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ทั้งสอง 2 ตัวหมุนต่อในทิศทางเดิมจากตำแหน่งเดิมเป็นเวลาอีก 20 วินาที
    {
      light_state = 0;
      ccw = 20;
    }

    // motor control
    if (cw)
    {
      motorCW();
      cw--;
    }
    else if (ccw)
    {
      motorCCW();
      ccw--;
    }
    else
    {
      motorStop();
    }
  }
}

void motorCW()
{
  digitalWrite(MOTOR1, HIGH);
  digitalWrite(MOTOR2, LOW);
  Blynk.virtualWrite(V3, 1);
  Blynk.virtualWrite(V4, 0);
}

void motorCCW()
{
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, HIGH);
  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 1);
}

void motorStop()
{
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);
  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 0);
}