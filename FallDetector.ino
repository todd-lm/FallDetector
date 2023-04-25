// Detects when user falls and triggers alarm/sends aler

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

const int buzzerPin = 14;

// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 30
#define initialThreshold 25

// Set threshold for movement detection
const int afterFallThreshold = 15;

void setup(void) {
#ifndef ESP8266
  while (!Serial) yield();     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
    Serial.println("Adafruit LIS3DH Tap Test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  lis.setClick(2, CLICKTHRESHHOLD);


  // Initialize buzzer
  pinMode(buzzerPin, OUTPUT);


  delay(100);
}


void loop() {
 
 uint8_t fall = 0;
 sensors_event_t event;
    lis.getEvent(&event);
  Serial.println(pow((pow(event.acceleration.x,2) + pow(event.acceleration.y,2) + pow(event.acceleration.z,2)), .5));
 if (pow((pow(event.acceleration.x,2) + pow(event.acceleration.y,2) + pow(event.acceleration.z,2)), .5) > initialThreshold) fall == 1;
 else {
  fall == 0;
  return;
 }


// uint8_t fall = lis.getClick();
//  if (! (fall & 0x30)) return;
//  Serial.print("Fall detected (0x"); Serial.print(fall, HEX); Serial.print("): ");
//  if (fall & 0x10) Serial.print(" single click");
//  if (fall & 0x20) Serial.print(" double click");
//    if (!fall) return;
//   Serial.println();

  //Add timer here to detect if theres no movement for a few seconds,  if there isn't assume user has fallen



  tone(buzzerPin, 440, 200);
  delay(600);
  

  // turn off tone function for pin 6:
  noTone(6);
  delay(2000);

  // Monitor accelerometer for 10 seconds
  unsigned long startTime = millis();
  
  bool fallDetected = true;

  while (millis() - startTime < 3000) {
    //sensors_event_t event;
    lis.getEvent(&event);
    if (abs(event.acceleration.x) > afterFallThreshold || abs(event.acceleration.y) > afterFallThreshold || abs(event.acceleration.z) > afterFallThreshold) {
 
      Serial.print("Alarm Cleared");
      Serial.println();
      fallDetected = false;
      return;

    }
    yield();
  }

if (fallDetected == true) {
        // Movement detected, trigger buzzer alarm
      Serial.print("ALARM ALARM ALARM");
      Serial.println();
for (int i = 0; i < 5; i++) {
      tone(buzzerPin,440,200);
   delay(300);
  noTone(buzzerPin);
  tone(buzzerPin,494,500);
  delay(300);
  noTone(buzzerPin);
  tone(buzzerPin,523,300);
   delay(300);
  noTone(buzzerPin);
}
}


  delay(100);
  return;
}
