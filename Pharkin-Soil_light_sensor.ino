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
#define BLYNK_TEMPLATE_ID "TMPLBPycYavo"              // จากหน้า blynk template
#define BLYNK_DEVICE_NAME "soil light"                // จากหน้า blynk template

#define BLYNK_FIRMWARE_VERSION        "0.0.1"

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

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h"    //blynk ติดตังจาก library manager
#include <BH1750FVI.h>      // BH1750FVI ของ PeterEmbedded ติดตังจาก library manager

// Create the Lightsensor instance
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);

unsigned long previousMillis = 0;
uint16_t lux, moisture;

void setup()
{
  Serial.begin(115200);
  LightSensor.begin();
  delay(100);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {   //ทำงานทุก 1 วินาที
    previousMillis = currentMillis;

    lux = LightSensor.GetLightIntensity();
    Serial.print("Light: " + String(lux));

    moisture = map(analogRead(A0), 1024, 500, 0, 100);
    Serial.println("\tSoli Moisture : " + (String)moisture);


  }



}

