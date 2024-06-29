//  nRF Repeater Project -- Sensor Transmitter 2, Classroom
//
//  Includes a PIR sensor to detect people coming in, and a DHT11 sensor
//  to measure local temperature and humidity. An LED indicates when
//  the module detected a presense via the PIR sensor (double-flash) or
//  when sending the heartbeat signal. Battery voltage and battery-low
//  information is also sent.

//  Based on:
//  Arduino Wireless Communication Tutorial
//  by Dejan Nedelkovski, www.HowToMechatronics.com
// 
//  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
//
//

#include <nrf_repeater_project.h>
#include <DHT11.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//  Define IO pins for this project
#define PIRPIN    16    // PIR sensor output on D16
#define BATPIN    A4    // Battery voltage divider output on D18/A4
#define ACTLEDPIN 8     // Activity LED on D8
#define DHTPIN    2     // Pin for DHT11 temp/humidity sensor
#define NRFCEPIN  10    // Pin for nRF radio CE pin
#define NRFCSNPIN 9     // Pin for nRF radio CSN pin

//  Define other constants
#define HBTICKSTO  5000           // Heartbeat ticks until timout. 5000 ticks = ~ 10 seconds
#define BATLOWV 2.5               // Indicates the lower voltage before setting the SBATLOW warning bit

//  Instantiate DHT11 temp/humidity sensor and nRF radio
DHT11 dht11( DHTPIN );
RF24 radio( NRFCEPIN, NRFCSNPIN ); // CE, CSN     // NRF CE and CSN on pins D10 and D9

SensorStruct_t sensorData;        // Instantiate sensorData as a global

//  ---------------------------------------------------------------------------
//  flashLED
//  ---------------------------------------------------------------------------
void flashLED( int LEDPin, int times, int period )
{
  for( int x=0; x<times; x++ )
  {
    digitalWrite( LEDPin, HIGH );
    delay( period );
    digitalWrite( LEDPin, LOW );
    delay( period );
  }
}

//  ===========================================================================
//  setup
//  ===========================================================================

void setup()
{
  sensorData.sensorID = SENSOR2;    // SENSOR2 defined in nrf_repeater_project.h

  // pinMode( PIRPIN, INPUT );      // Not needed as Arduino IO pins default to input
  pinMode( ACTLEDPIN, OUTPUT );

  analogReference( INTERNAL1V1 );   // Set the internal 1.1 V reference

  flashLED( ACTLEDPIN, 3, 100 );    // Flash the LED to indicate the module is starting up
  
  radio.begin();                        // Start the radio
  radio.openWritingPipe(pipeSensor2);   // Set write pipe
  radio.setPALevel(RF24_PA_LOW);        // Set radio output level
  radio.setDataRate( RF24_250KBPS );    // Set data rate
  radio.stopListening();                // Stop listening because this module only transmits

} // End of setup block


//  ===========================================================================
//  loop
//  ===========================================================================
void loop()
{
  unsigned long startTick = ticks();    // To keep track of time for heartbeat
  unsigned long timePassed;             // Also to keep time, as above
  int pirGet;                           // To remember if PIR sensor triggered in loop
  int temp, humid;                      // Hold temperature and humidity values
  float batV;                           // Used to calculate and hold this modules battery voltage

  //  Wait here until either the PIR sensor is triggered or until a certain time has passed
  do
  {
    pirGet     = digitalRead( PIRPIN );             // Note that a LOW value indicates a detection
    timePassed = ticks() - startTick;
  }
  while( pirGet && ( timePassed < HBTICKSTO ) ) ;   // Wait for PIR input or heartbeat timeout to exit loop
  
  dht11.readTemperatureHumidity( temp, humid );     // Get temp/humid reading from DHT11 sensor
  sensorData.sensorTemp = temp;                     // Enter theese values into the data packet
  sensorData.sensorHumid = humid;
  
  if( !pirGet )                                   // If Got a ping on the sensor
  {
    sensorData.dataType = PING | TEMP | HUMID;    // Send ping, temp, and humid data.
    flashLED( ACTLEDPIN, 2, 10 );                 // Give a double flash to indicate activity
  }
  else
  {
    sensorData.dataType = HB | TEMP | HUMID;      // Send heartbeat, temp, and humid data.
    flashLED( ACTLEDPIN, 1,10 );
  }

  batV = 0;
  for( int x=0; x<10; x++ ) ;     // Let ADC settle
  for( int x=0; x<10; x++ )
  {
    batV += analogRead( BATPIN );
  }
  batV /= (float)10;
  sensorData.sensorBatV = (float)batV/207.778+0.1294;

  if( batV < BATLOWV )
    sensorData.dataType |= SBATLOW;   // Set battery low bit if battery voltage is low

  radio.write(&sensorData, sizeof(sensorData));
  delay(5000);
}