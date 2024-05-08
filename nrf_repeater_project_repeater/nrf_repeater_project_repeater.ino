/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(10, 9); // CE, CSN
#define ACTLED 8

const uint64_t pipe1 = 0xF0F0F0F0A1;
const uint64_t pipe2 = 0xF0F0F0F0A2;

char text[32]="";


void setup() {
  pinMode(ACTLED, OUTPUT);

  radio.begin();
  radio.openReadingPipe( 1, pipe1 );
  radio.openWritingPipe( pipe2 );
  radio.setPALevel( RF24_PA_MAX );
  radio.setDataRate( RF24_250KBPS );
}

void loop() {
  delay( 5 );
  radio.startListening();
  while( !radio.available()) ;
  radio.read( &text, sizeof(text) );
  delay( 5 );
  digitalWrite( ACTLED, HIGH );
  delay( 250 );
  digitalWrite( ACTLED, LOW );
  radio.stopListening();
  radio.write( &text, sizeof(text) );
}