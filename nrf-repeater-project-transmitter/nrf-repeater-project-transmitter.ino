/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <nrf_repeater_project.h>
#include <DHT11.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PIR 15                    // PIR output on D15
#define BATPIN A2

DHT11 dht11( 14 );

RF24 radio(10, 9); // CE, CSN     // NRF CE and CSN on pins D10 and D9

SensorStruct_t sensorData;          // Instantiate sensorData as a global

void setup()
{
  sensorData.sensorID = SENSOR1;          // This device has ID = 1

  pinMode( PIR, INPUT );          // Define PIR pin as input
  analogReference( INTERNAL1V1 );

  radio.begin();
  radio.openWritingPipe(pipeSensor1);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate( RF24_250KBPS );
  radio.stopListening();

}

void loop() {
  unsigned long startTick = ticks();
  unsigned long timePassed;
  int pirGet;
  int temp, humid;
  float batV;

  do
  {
    pirGet = digitalRead( PIR );
    timePassed = ticks() - startTick;
  }
  while( pirGet && (timePassed < SensorHBTicksTO) ) ;       // Wait for PIR input
  
  dht11.readTemperatureHumidity( temp, humid );
  sensorData.sensorTemp = temp;
  sensorData.sensorHumid = humid;
  
  if( !pirGet )                                   // Got a ping on the sensor
  {
    sensorData.dataType = PING | TEMP | HUMID;    // Send ping, temp, and humid data.
  }
  else
  {
    sensorData.dataType = HB | TEMP | HUMID;      // Send heartbeat, temp, and humid data.
  }

  
  batV = 0;
  for( int x=0; x<10; x++ ) ;     // Let ADC settle
  for( int x=0; x<10; x++ )
  {
    batV += analogRead( BATPIN );
  }
  batV /= (float)10;
  sensorData.batV = (float)batV/230.0+0.4174;

  radio.write(&sensorData, sizeof(sensorData));
  delay(5000);
}