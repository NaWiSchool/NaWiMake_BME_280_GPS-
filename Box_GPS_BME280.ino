/*
   Lernkarte_BME280_GPS - Lernkarte_BME280_GPS.cpp
   [This code is used to read and save the data from a BME280 sensor and a GT-U7 GPS module]

    Created on: 13/08/2020
    Author: Schellenberg, Fabian
    All rights reserved. Copyright Fabian Schellenberg 2020.
*/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>

File dataFile;
int pinCS = 10;
int d = 0;
int e = 0;
#define LED 2

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

LiquidCrystal_I2C lcd(0x27, 16, 2);

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

Adafruit_BME280 bme; // I2C

bool failflag = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  ss.begin(GPSBaud);

  if (!SD.begin(pinCS))
  {
    Serial.println("Card failed, or not present");
  }
  d = 0;

  unsigned status;
  status = bme.begin();
  if (!status) {
    failflag = true;
  }

  

  lcd.begin();
  lcd.backlight();

  Serial.println(F("Lernkarte_GPS.ino"));
  Serial.print(F("Code written by Fabian Schellenberg"));
  Serial.println();


}

void loop() {

  while (ss.available() > 0)
    gps.encode(ss.read());
  delay(50);
  
  if (gps.time.isUpdated()) {
    e++;
    if (e > 10) {
      sdwrite();
      e = 0;
    }
  }

}

void sdwrite() {

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    if (d == 0) {
      dataFile.println("Neue Messung:");
      dataFile.println("lat, lng, alt, hh:mm:ss, temp, press, hum");
      d = 1;
    }
    dataFile.print(String(gps.location.lat(), 6));
    dataFile.print(", ");
    dataFile.print(String(gps.location.lng(), 6));
    dataFile.print(", ");
    dataFile.print(String(gps.altitude.meters()));
    dataFile.print(", ");
    dataFile.print(gps.time.hour());
    dataFile.print(":");
    dataFile.print(gps.time.minute());
    dataFile.print(":");
    dataFile.print(gps.time.second());
    dataFile.print(", ");
    dataFile.print(bme.readTemperature());
    dataFile.print(", ");
    dataFile.print(bme.readPressure() / 100.0F);
    dataFile.print(", ");
    dataFile.print(bme.readHumidity());
    dataFile.println(", ");
    dataFile.close();
  }
  Serial.println(gps.time.second());
  Serial.println(String(gps.altitude.meters()));
  Serial.println(failflag);

 if (gps.altitude.meters() && bme.readTemperature() != 0){
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);
 }
 else{
  digitalWrite(LED, HIGH);
 }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS:Alt= ");
  lcd.setCursor(9, 0);
  lcd.print(String(gps.altitude.meters()));
  lcd.setCursor(0, 1);
  lcd.print("BME:Temp= ");
  lcd.setCursor(10, 1);
  lcd.print(bme.readTemperature());


}
