/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <nrf_repeater_project.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(10, 9); // CE, CSN
#define ACTLED 8

SensorStruct_t sensorData;


void setup() {
  sensorData.sensorID = REPEATER1;

  pinMode(ACTLED, OUTPUT);

  radio.begin();
  radio.openReadingPipe( 1, pipeSensor1 );
  radio.openWritingPipe( pipeReceiver );
  radio.setPALevel( RF24_PA_MAX );
  radio.setDataRate( RF24_250KBPS );

  for( int x=0; x<5; x++ )
  {
    digitalWrite( ACTLED, HIGH );
    delay(100);
    digitalWrite(ACTLED, LOW);
    delay(100);
  }
}

void loop() {
  delay( 5 );
  radio.startListening();
  delay( 5 );
  while( !radio.available()) ;
  radio.read( &sensorData, sizeof( sensorData ) );
  digitalWrite( ACTLED, HIGH );
  if( !(sensorData.dataType & HB) )
  {
    digitalWrite( ACTLED, HIGH );   // Show a short pulse for the heartbeat
    delay( 50 );
    digitalWrite( ACTLED, LOW );
  }
  delay( 50 );
  digitalWrite( ACTLED, HIGH );
  delay( 50 );
  digitalWrite( ACTLED, LOW );
  radio.stopListening();
  radio.write( &sensorData, sizeof( sensorData ) );
}