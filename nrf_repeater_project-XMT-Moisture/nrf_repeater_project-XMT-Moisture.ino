//  nRF Repeater Project -- Moisture Sensor Transmitter 3, Garden
//
//  Includes a soil moisture sensor to detect relative soil moisture.
//  Also includes a DHT11 temperature and humidity sensor to test ambient
//  temp and humidity near the ground (or wherever the sensor is located).
//  The activity LED will blink to indicate data being sent and can be used
//  for other diagnostics. Will flash briefly when the module is powered up.
//  Battery voltage and battery-low information is also sent.

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
#define MOISTPIN  A3    // Moisture sensor output on P26/D17/A3
#define BATPIN    A4    // Battery voltage divider output on P27/D18/A4
#define ACTLEDPIN 8     // Activity LED on P14/D8
#define DHTPIN    14    // Pin for DHT11 temp/humidity sensor P23/D14/A0
#define NRFCEPIN  10    // Pin for nRF radio CE pin P16/D10
#define NRFCSNPIN 9     // Pin for nRF radio CSN pin P15/D9

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
  sensorData.sensorID = SENSOR3;    // SENSOR2 defined in nrf_repeater_project.h

  // pinMode( MOISTPIN, INPUT );    // Not needed as Arduino IO pins default to input
  pinMode( ACTLEDPIN, OUTPUT );

  analogReference( INTERNAL1V1 );   // Set the internal 1.1 V reference

  flashLED( ACTLEDPIN, 3, 100 );    // Flash the LED to indicate the module is starting up
  
  radio.begin();                        // Start the radio
  radio.openWritingPipe(pipeSensor3);   // Set write pipe
  radio.setPALevel(RF24_PA_LOW);        // Set radio output level
  radio.setDataRate( RF24_250KBPS );    // Set data rate
  radio.stopListening();                // Stop listening because this module only transmits

} // End of setup block


//  ===========================================================================
//  loop
//  ===========================================================================
void loop()
{
  int moistGet;                         // To hold current moisture reading (as a percent)
  int temp, humid;                      // Hold temperature and humidity values
  float batV;                           // Used to calculate and hold this modules battery voltage

  for( int x=00; x<10; x++ )
    analogRead( MOISTPIN );

  moistGet=0;
  for( int x=0; x<10; x++ )
    moistGet += analogRead( MOISTPIN );
  
  moistGet /= (float)10;
  sensorData.sensorMoist = moistGet;
  

  dht11.readTemperatureHumidity( temp, humid );     // Get temp/humid reading from DHT11 sensor
  sensorData.sensorTemp = temp;                     // Enter theese values into the data packet
  sensorData.sensorHumid = humid; 
  
  sensorData.dataType = HB | TEMP | HUMID | MOIST;  // Send heartbeat, temp, and humid data.
  flashLED( ACTLEDPIN, 1, 10 );

  batV = 0;
  for( int x=0; x<10; x++ )
    analogRead( BATPIN );     // Let ADC settle
  
  batV = 0;
  for( int x=0; x<10; x++ )
  {
    batV += analogRead( BATPIN );
  }
  batV /= (float)10;
  sensorData.sensorBatV = (float)batV/197.059 + 0.1994;
  //sensorData.sensorBatV = float((int(batV*10))/10);

  if( batV < BATLOWV )
    sensorData.dataType |= SBATLOW;   // Set battery low bit if battery voltage is low

  radio.write(&sensorData, sizeof(sensorData));
  delay(5000);
}