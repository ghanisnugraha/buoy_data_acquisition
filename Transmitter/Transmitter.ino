#include <DS3231.h>
#include <TimeLib.h>
#include <MPU6050.h>
#include <SX126XLT.h>
#include <QMC5883L.h>
#include <TinyGPS++.h>
#include <SimpleTimer.h>
#include <DallasTemperature.h>

#define SD_cs 48
#define temp1_pin 47
#define temp2_pin 49
#define windDir_pin A1
#define windSpd_pin A0

#define NSS 46
#define NRESET 42
#define RFBUSY 44
#define RX_EN 36
#define TX_EN 34

#define LORA_DEVICE DEVICE_SX1262
#define TXpower 20

OneWire temp_ow1(temp1_pin), temp_ow2(temp2_pin);
DallasTemperature temp_sensor1(&temp_ow1), temp_sensor2(&temp_ow2);
MPU6050 IMU;
TinyGPSPlus gps;
SimpleTimer dataTimer;
SX126XLT LoRa;
QMC5883L compass;

union {
  struct {
    uint32_t timestamp;
    float wind_direction;
    float wind_speed;
    float temperature;
    float wave_height;
    float latitude;
    float longitude;
  } parameter;
  byte packet[28];
} data;

float temp_data1, temp_data2;
String dataString;
tmElements_t timeNow;
Vector accel;
float accelOffset;
float accelSum = 0;
float accelBuffer[250];
float integral = 0;
float doubleIntegral = 0;
float maxIntegral = 0;
float compassData = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial1.begin(38400);
  Serial.begin(57600);
  Serial.println("Initializing...");
  
  Wire.begin();
  compass.init();
  compass.setSamplingRate(50);
  compass.setCalibration(-5580, 7470, -3680, 6100);

  while (!LoRa.begin(NSS, NRESET, RFBUSY, -1, -1, -1, RX_EN, TX_EN, -1, LORA_DEVICE)) error();
  LoRa.setupLoRa(924000000, 0, LORA_SF7, LORA_BW_250, LORA_CR_4_5, LDRO_AUTO);

  temp_sensor1.begin();
  temp_sensor2.begin();

  while (!IMU.begin(0, MPU6050_RANGE_2G)) error();
  for (int i = 0; i < 250; i++) {
    accel = IMU.readNormalizeAccel();
    accelBuffer[i] = accel.ZAxis;
    delay(10);
  }
  for (int i = 0; i < 250; i++) {
    accelSum += accelBuffer[i];
  }
  accelOffset = accelSum / 250;

  delay(100);
  dataTimer.setInterval(10000, dataTimerHandler);

  Serial.println("Ready");
}

void loop() {
  dataTimer.run();

  if (Serial1.available()) gps.encode(Serial1.read());
}

void dataTimerHandler() {
  getGPSData();
  data.parameter.wind_direction = getWindDir();
  data.parameter.wind_speed = getWindSpeed();
  data.parameter.temperature = getTempData();
  data.parameter.wave_height = getWaveHeight();
  
  if ((data.parameter.timestamp != 0) && (data.parameter.latitude != 0.0) && (data.parameter.longitude != 0.0)) sendData();
}

void sendData() {
  LoRa.transmit(data.packet, sizeof(data.packet), 3000, TXpower, NO_WAIT);
}

void getGPSData() {
  if (gps.location.isUpdated()) {
    data.parameter.latitude = gps.location.lat();
    data.parameter.longitude = gps.location.lng();
  }
  
  if (gps.time.isValid() && gps.date.isValid()) {
    timeNow.Second = gps.time.second();
    timeNow.Minute = gps.time.minute();
    timeNow.Hour = gps.time.hour();
    timeNow.Day = gps.date.day();
    timeNow.Month = gps.date.month();
    timeNow.Year = gps.date.year() - 1970;

    data.parameter.timestamp = makeTime(timeNow);
  }
}

float getTempData() {
  temp_sensor1.requestTemperatures();
  temp_sensor2.requestTemperatures();
  delay(10);
  temp_data1 = temp_sensor1.getTempCByIndex(0);
  temp_data2 = temp_sensor2.getTempCByIndex(0);
  return ((temp_data1 + temp_data2) / 2);
}

void getCompassData() {
  compassData = (float)compass.readHeading();
}

float getWindDir() {
  float raw = (float)analogRead(windDir_pin);
  float heading = raw * 359 / 1023 + compassData;

  if (heading > 720) heading = heading - 720;
  else if (heading > 360) heading = heading - 360;
  else if (heading < 0) heading = heading + 360;

  return heading;
}

float getWindSpeed() {
  int raw = analogRead(windSpd_pin);
  return (float)raw * 30 / 1023;
}

float getWaveHeight() {
  integral = 0;
  doubleIntegral = 0;
  maxIntegral = 0;
  for (int i = 0; i < 250; i++) {
    accel = IMU.readNormalizeAccel();
    accelBuffer[i] = accel.ZAxis - accelOffset;
    delay(20);
  }
  for (int i = 0; i < 250; i++) {
    integral += accelBuffer[i] * 0.02;
    doubleIntegral += integral * 0.02;
    if (doubleIntegral > maxIntegral) maxIntegral = doubleIntegral;
  }
  return maxIntegral;
}

void error() {
  Serial.println("Sensor Error");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}
