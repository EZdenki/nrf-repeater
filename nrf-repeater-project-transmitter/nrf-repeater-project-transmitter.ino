/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PIR 15                    // PIR output on D15

RF24 radio(10, 9); // CE, CSN     // NRF CE and CSN on pins D10 and D9

const uint64_t pipe1 = 0xF0F0F0F0A1;
char text[32] = "Hello World!";

void setup() {

  pinMode( PIR, INPUT );          // Define PIR pin as input

  radio.begin();
  radio.openWritingPipe(pipe1);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate( RF24_250KBPS );

  radio.stopListening();
}

void loop() {
  while( !digitalRead( PIR ) ) ;       // Wait for PIR input

  radio.write(&text, sizeof(text));
  delay(5000);
}