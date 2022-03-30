#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 9);
const byte address[6] = "01234";

bool light = false;
int mx, my, rot, alt;

void setup() {
  Serial.begin(38400);
  pinMode(5, OUTPUT);
  Serial.flush();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(true);
  radio.setPayloadSize(10);
  radio.stopListening();
}

void loop() {
  static byte packet[10];
  //Serial.println("Test");
  if (Serial.available() > 0) {
    Serial.readBytes(packet, 10); // read full packet
    //radio.write(&packet, 10); // Send packet through radio transmitter
	  radio.writeFast(&packet, 10); // Send packet through radio transmitter
    
    byte mask = packet[0];
    if((mask & 0b10000000) > 0) {
      if((mask & 0b00000001) > 0) {
        mx = (packet[1] << 8) | packet[2];
        Serial.print(" mx: ");
        Serial.print(mx);
      }
      if((mask & 0b00000010) > 0) {
        my = (packet[3] << 8) | packet[4];
        Serial.print(" my: ");
        Serial.print(my);
      }
      if((mask & 0b00000100) > 0) {
        rot = (packet[5] << 8) | packet[6];
        Serial.print(" rot: ");
        Serial.print(rot);
      }
      if((mask & 0b00001000) > 0) {
        alt = (packet[7] << 8) | packet[8];
        analogWrite(5, (alt + 512)/4);
        Serial.print(" alt: ");
        Serial.print(alt);
      }
      if((mask & 0b00010000) > 0) {
        light = packet[9];
        //digitalWrite(5, light);
        Serial.print(" Light: ");
        Serial.print(light);
      }
      Serial.println();
    }
  }
  //while(Serial.available() > 
}
