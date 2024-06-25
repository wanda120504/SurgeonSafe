#include <ESP32Firebase.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MQUnifiedsensor.h"

#define Board ("ESP-32")
#define Pin (34)
#define Type ("MQ-2")
#define Voltage_Resolution (3.3)
#define ADC_Bit_Resolution (12)
#define RatioMQ135CleanAir (3.6)

MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//define pin
#define RELAY_PIN 5  // Ganti dengan pin relay pada ESP32
#define RED_PIN 13  // Ganti dengan pin relay pada ESP32
#define GREEN_PIN 12  // Ganti dengan pin relay pada ESP32
Adafruit_BME280 bme; // I2C


//define wifi
#define WIFI_SSID "Rehat Dulu"
#define WIFI_PASSWORD "12345678"

#define API_KEY "AIzaSyC81uLGToGjwvb5fpu95-_5vOJKK3inj1s"

#define DATABASE_URL "https://kelompok8-f43c7-default-rtdb.firebaseio.com/"

Firebase firebase(DATABASE_URL);

void setup() {
  //initialize serial
  Serial.begin(9600);
  delay(500);

  //initialize bme sensor
  if (!bme.begin(0x76)) {   // Set the address of your BME280 here. Default is 0x77.
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  //initialize wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP :");
  Serial.println(WiFi.localIP());
  Serial.println();

  //pinmode
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  //initialize mq135
  MQ135.setRegressionMethod(1);
  MQ135.setA(110.47);
  MQ135.setB(-2.862);

  MQ135.init();
  MQ135.setRL(10);

  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0 / 10);

  if (isinf(calcR0)) {
    Serial.println("Peringatan: Masalah koneksi, R0 tak terhingga (Sirkuit terbuka terdeteksi) harap periksa kabel dan suplai");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Peringatan: Masalah koneksi ditemukan, R0 adalah nol (Pin analog korsleting ke tanah) harap periksa kabel dan suplai");
    while (1);
  }
}

void sendToFirebase(float ppm, float temperature, float humidity) {
    // Set data
    firebase.setFloat("/DATA/temperature", temperature);
    firebase.setFloat("/DATA/humidity", humidity);
    firebase.setFloat("/DATA/ppm", ppm);
}

void bmp() {
  float temperature = bme.readTemperature(); // Get temperature from BME280
  float humidity = bme.readHumidity(); // Get humidity from BME280
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
}

void mq() {
  MQ135.update();
  float ppm = MQ135.readSensor();
  Serial.print(ppm);
  Serial.println(" PPM");

  int value = analogRead(Pin);
  value = map(value, 0, 4095, 0, 100);
}

void loop(){
  bmp();
  mq();
  delay(1000);
  float ppm = MQ135.readSensor();
  float temperature = bme.readTemperature(); // Get temperature from BME280
  float humidity = bme.readHumidity(); // Get humidity from BME280
  sendToFirebase(ppm, temperature, humidity);

  if(temperature < 15 || temperature > 30) {
  // Aksi yang diinginkan jika suhu kurang dari 15 atau lebih dari 30
  digitalWrite(RELAY_PIN, HIGH);
  Serial.print("HIGH");
  digitalWrite(GREEN_PIN, HIGH);
  firebase.setString("/DATA/Indikator", "Kualitas udara tidak aman! Operasi tidak bisa dilaksanakan");
  }

  else if(humidity < 30 || humidity > 70) {
  // Aksi yang diinginkan jika suhu kurang dari 15 atau lebih dari 30
  digitalWrite(RELAY_PIN, HIGH);
  Serial.print("HIGH");
  digitalWrite(GREEN_PIN, HIGH);
  firebase.setString("/DATA/Indikator", "Kualitas udara tidak aman! Operasi tidak bisa dilaksanakan");
  }

  else if(ppm > 2000) {
  // Aksi yang diinginkan jika suhu kurang dari 15 atau lebih dari 30
  digitalWrite(RELAY_PIN, HIGH);
  Serial.print("HIGH");
  digitalWrite(GREEN_PIN, HIGH);
  firebase.setString("/DATA/Indikator", "Kualitas udara tidak aman! Operasi tidak bisa dilaksanakan");
  }

  else {
  digitalWrite(RELAY_PIN, LOW);
  Serial.print("LOW");
  digitalWrite(GREEN_PIN, LOW);
  firebase.setString("/DATA/Indikator", "Kualitas udara aman! Operasi bisa dilaksanakan");
  }
}


