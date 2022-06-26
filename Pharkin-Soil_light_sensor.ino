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
#define BLYNK_TEMPLATE_ID "TMPLfHXtxAbw" // จากหน้า blynk template
#define BLYNK_DEVICE_NAME "Soil Moisture and Light sensor"   // จากหน้า blynk template

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
uint32_t lux;
uint16_t moisture;
bool relayOn = false;            // flag relay on
uint8_t cw, ccw;                 // timer to control motor
uint8_t light_state, mois_state; // state
uint8_t timer;

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
  if (currentMillis - previousMillis >= 100)
  { //ทำงานทุก 0.1 วินาที เพื่อหาค่าเฉลี่ย 10 ค่า
    previousMillis = currentMillis;
    lux += LightSensor.GetLightIntensity();
    moisture += map(analogRead(A0), 1024, 500, 0, 100);
    timer++;
  }

  if (timer >= 10) //ทำงานทุก 1 วินาที
  {
    // ค่าเฉลี่ยแสง
    lux = lux / timer;
    Serial.print("Light: " + String(lux));
    // ค่าเฉลี่ยความชื้น
    moisture = moisture / timer;
    Serial.println("\tSoli Moisture : " + (String)moisture);

    // update blynk
    Blynk.virtualWrite(V0, moisture);
    Blynk.virtualWrite(V1, lux);

    //ข้อ1 เงื่อนไขจากความชื้น
    if (moisture <= MIN_MOISTURE)
    {
      digitalWrite(RELAY, HIGH);
      relayOn = true;
      Serial.println("Relay On");
    }
    else if (moisture > MAX_MOISTURE)
    {
      digitalWrite(RELAY, LOW);
      relayOn = false;
      Serial.println("Relay Off");
    }
    Blynk.virtualWrite(V2, relayOn);

    //ข้อ2 เงื่อนไขจากแสง
    if (light_state == 0 && lux > MAX_LUX && cw == 0 && ccw == 0) //ตรวจจับได้ว่าค่าความเข้มแสงมีค่าเกินกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ตัวหนึ่งหมุนตามเข็มนาฬิกาและมอเตอร์อีกตัวหนึ่งหมุนทวนเข็มนาฬิกาเป็นเวลา 20 วินาที
    {
      light_state = 1;
      cw = 20;
      Serial.println("Light > " + String(MAX_LUX) + " -> motor : Clockwise  Time:" + String(cw) + " sec");
    }
    else if (light_state == 1 && lux < MIN_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงตรวจจับได้ว่าค่าความเข้มแสงมีค่าต่ำกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ทั้งสอง 2 ตัวหมุนต่อในทิศทางเดิมจากตำแหน่งเดิมเป็นเวลาอีก 20 วินาที
    {
      light_state = 2;
      cw = 20;
      Serial.println("Light < " + String(MIN_LUX) + " -> motor : Clockwise  Time:" + String(cw) + " sec");
    }
    else if (light_state == 2 && lux > MAX_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงสามารถตรวจจับได้ว่าค่าความเข้มแสงนั้นมีค่ามากกว่าช่วงที่กำหนดอีกครั้งให้ทำการสั่งการให้มอเตอร์หมุนทวนเข็มนาฬิกาเป็นเวลา 30 วินาที
    {
      light_state = 3;
      ccw = 30;
      Serial.println("Light > " + String(MAX_LUX) + " -> motor : CounterClockwise  Time:" + String(ccw) + " sec");
    }
    else if (light_state == 3 && lux < MIN_LUX && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความเข้มแสงตรวจจับได้ว่าค่าความเข้มแสงมีค่าต่ำกว่าช่วงที่กำหนดไว้ให้ทำการสั่งการให้มอเตอร์ทั้งสอง 2 ตัวหมุนต่อในทิศทางเดิมจากตำแหน่งเดิมเป็นเวลาอีก 20 วินาที
    {
      light_state = 0;
      ccw = 20;
      Serial.println("Light < " + String(MIN_LUX) + " -> motor : CounterClockwise  Time:" + String(ccw) + " sec");
    }
    //ข้อ3 เงือนไขจากความชื้น จะทำงานเมื่อไม่เข้าเงือนไขในข้อ2
    else if (mois_state == 0 && moisture > MAX_MOISTURE && cw == 0 && ccw == 0) // หากเซนเซอร์ตรวจจับความชื้นภายในดินตวจจับได้ว่าค่าความชื้นภายในดินมีค่าเกินกว่าที่กำหนดให้ทำการสั่งการให้มอเตอร์ตัวหนึ่งหมุนตามเข็มนาฬิกา เป็นเวลา 20 วินาที
    {
      mois_state = 1;
      cw = 20;
      Serial.println("Moisture > " + String(MAX_MOISTURE) + " -> motor : Clockwise  Time:" + String(cw) + " sec");
    }
    else if (mois_state == 1 && moisture <= MAX_MOISTURE && moisture >= MIN_MOISTURE && cw == 0 && ccw == 0) // ต่อมาหากเซนเซอร์ตรวจจับความชื้นภายในดินตรวจจับได้ว่าค่าความชื้นภายในดินกลับมาอยู่ในช่วงที่กำหนดไว้ ให้ทำการสั่งการให้มอเตอร์หมุนทวนเข็มนาฬิกา เป็นเวลา 30 วินาที
    {
      mois_state = 0;
      ccw = 30;
      Serial.println("Moisture in " + String(MIN_MOISTURE) + '-' + String(MAX_MOISTURE) + " -> motor : CounterClockwise  Time:" + String(ccw) + " sec");
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

    // clear value
    timer = 0;
    lux = 0;
    moisture = 0;
  }
}

void motorCW() // motor : Clockwise
{

  digitalWrite(MOTOR1, HIGH);
  digitalWrite(MOTOR2, LOW);
  Blynk.virtualWrite(V3, 1);
  Blynk.virtualWrite(V4, 0);
}

void motorCCW() // motor : Counterclockwise
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
