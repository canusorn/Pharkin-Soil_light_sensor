/*
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

    // เงื่อนไขจากแสง
    if (lux > MAX_LUX)
    {
    }
    else if (lux < MIN_LUX)
    {
    }
  }
}

void motorCW()
{
  digitalWrite(MOTOR1, HIGH);
  digitalWrite(MOTOR2, LOW);
}

void motorCCW()
{
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, HIGH);
}

void motorStop()
{
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);
}