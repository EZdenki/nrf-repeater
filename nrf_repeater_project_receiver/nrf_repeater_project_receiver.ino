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
#include <toneAC.h>

void tone1( void )
{
  
  for(int x=0; x<3; x++)
  {
    for( int tone=400; tone<2000; tone ++)
    {
      toneAC(tone);
      //delay(2);
    }
        for( int tone=2000; tone>400; tone --)
    {
      toneAC(tone);
      //delay(2);
    }
    toneAC(0);
  }
}


RF24 radio(7, 8); // CE, CSN
#define BUZZER 9
const uint64_t pipe2 = 0xF0F0F0F0A2;
char text[32]="";


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipe2);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate( RF24_250KBPS );
  radio.startListening();
  tone1();
} 

void loop() {
  if (radio.available()) {
    radio.read(&text, sizeof(text));
    tone1();
    Serial.println(text);
  }
}