#include "heltec.h"
#include <Wire.h>
#include <AS5600.h>
#include <can.h>
#include <mcp2515.h>

struct can_frame canMsg1;
//HiLetgo 2pcs MCP2515 CAN Bus Module TJA1050 Receiver SPI Module for Arduino AVR 
//https://www.amazon.com/gp/product/B01D0WSEWU/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
//MakerFocus ESP32 Development Board Upgraded Version 8MB Flash, ESP32 WiFi Bluetooth, ESP32 OLED 0.96 Inch OLED Display CP2102 Internet for Arduino ESP8266 NodeMCU 
//https://www.amazon.com/gp/product/B076KJZ5QM/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
//12bit Precision AS5600 Encoder Magnetic Induction Angle Measuring Sensor Module
//https://www.ebay.com/itm/12bit-Precision-AS5600-Encoder-Magnetic-Induction-Angle-Measuring-Sensor-Module/264739232981?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649
//
//AS5600
//SCL-15,SDA-4

//MCP2515
//CS-14,SCK-18,MOSI-23,MISO-19

MCP2515 can1(14);

int32_t encoder0Reading = 0;
int32_t lastencoder0Reading = 0;
int32_t rate0 = 0;

int32_t encoder1Reading = 0;
int32_t lastencoder1Reading = 0;
int32_t rate1 = 0;

int32_t start = 0;
int32_t counter=0;

AMS_5600 ams5600;

int32_t ang, realang=0;

void setup()
{
  //SPI.begin(18, 19, 23, 14);

  Serial.begin(115200);
  Heltec.begin(true, false, true);
  Wire.begin(SDA_OLED, SCL_OLED);
  
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);

  can1.reset();
  can1.setBitrate(CAN_500KBPS, MCP_8MHZ);
  can1.setNormalMode();
  delay(10);
  start = ams5600.getRawAngle();
  if(start>1000)
    counter=1;
}

void loop()
{
    Heltec.display->clear();
    encoder0Reading = ams5600.getRawAngle();
    rate0 = abs(encoder0Reading) - abs(lastencoder0Reading);
    lastencoder0Reading = encoder0Reading;
    if(rate0>1000){
      rate0=0;
      counter--;
     }

    if(rate0<-1000){
      rate0=0;
      counter++;
     }

  encoder1Reading=encoder0Reading+counter*4096;
    rate1 = abs(encoder1Reading) - abs(lastencoder1Reading);
    lastencoder1Reading = encoder1Reading;

    if(rate1>1000){
      rate1=0;
     }

    if(rate1<-1000){
      rate1=0;
     }
  
    canMsg1.can_id  = 0x23;
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = (encoder1Reading >> 24) & 0xFF; //Bitshift the ANGSENSOR (Cabana errors with 32 bit)
    canMsg1.data[1] = (encoder1Reading >> 16) & 0xFF;
    canMsg1.data[2] = (encoder1Reading >> 8) & 0xFF;
    canMsg1.data[3] = (encoder1Reading >> 0) & 0xFF;
    canMsg1.data[4] = (rate1 >> 16) & 0xFF;
    canMsg1.data[5] = (rate1 >> 8) & 0xFF;
    canMsg1.data[6] = (rate1 >> 0) & 0xFF;
    canMsg1.data[7] = can_cksum (canMsg1.data, 7, 0x230); //Toyota CAN CHECKSUM

    can1.sendMessage(&canMsg1);
    
    Heltec.display->drawString(0, 10, String(counter));
    Heltec.display->drawString(0, 20, String(encoder1Reading));
    Heltec.display->drawString(0, 30, String(rate1));

    Heltec.display->display();
      
    delay(10);
}

int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}
