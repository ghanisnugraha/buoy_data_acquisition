#include <SPI.h>
#include <SX126XLT.h>

#define NSS PA4
#define NRESET PB0
#define RFBUSY PB1
#define RX_EN PC14
#define TX_EN PC15
#define DIO1 PB10
#define DIO2 PB11
#define LORA_DEVICE DEVICE_SX1262

SX126XLT LoRa;
uint8_t RXPacketL;

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

void setup() {
  Serial.begin(9600);

  SPI.begin();
  delay(200);
  if (!LoRa.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, -1, RX_EN, TX_EN, -1, LORA_DEVICE))
  {
    Serial.println("No device responding");
    while (1);
  }

  LoRa.setupLoRa(924000000, 0, LORA_SF7, LORA_BW_250, LORA_CR_4_5, LDRO_AUTO);
  delay(3000);
}

void loop() {
  RXPacketL = LoRa.receive(data.packet, sizeof(data.packet), 5000, WAIT_RX);

  if (RXPacketL != 0) {
    Serial.write(data.packet, sizeof(data.packet));
  }

  delay(1000);
}

void printData() {
  Serial.print(data.parameter.timestamp); Serial.print("; ");
  Serial.print(data.parameter.wind_direction); Serial.print("; ");
  Serial.print(data.parameter.wind_speed); Serial.print("; ");
  Serial.print(data.parameter.light_intensity); Serial.print("; ");
  Serial.print((float)data.parameter.temperature); Serial.print("; ");
  Serial.print((float)data.parameter.wave_height); Serial.print("; ");
  Serial.print((float)data.parameter.latitude, 6); Serial.print("; ");
  Serial.print((float)data.parameter.longitude, 6);
  Serial.println();
}
