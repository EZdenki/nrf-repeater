//  nRF Repeater Project -- Sensor Transmitter 1, Front Door
//
//  Includes an HC-SR04 ultrasonic proximity sensor to detect people coming in,
//  and a DHT11 sensor to measure local temperature and humidity. An LED indicates
//  when the module detected a presense via the PIR sensor (double-flash) or when
//  when sending the heartbeat signal. Battery voltage and battery-low information
//  is also sent.

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
#define PINGTRIGPIN 18
#define PINGECHOPIN 19
#define BATPIN      A2
#define DHTPIN      14
#define NRFCEPIN    10
#define NRFCSNPIN   9

//  Define other constants
#define HBTICKSTO  5000           // Heartbeat ticks until timout. 5000 ticks = ~ 10 seconds
#define PINGDIST   200            // Will ping if detected movement within this distance in cm
#define BATLOWV    2.9            // Ultrasonic sensor misfires when below this voltage
                                  // so do not send "PING" signal and do indicate battery low
                                  // to receiver.


//  Instantiate DHT11 temp/humidity sensor and nRF radio
DHT11 dht11( DHTPIN );              // Instatiate temp/humidity object
RF24 radio( NRFCEPIN, NRFCSNPIN );  // CE, CSN     // NRF CE and CSN on pins D10 and D9

SensorStruct_t sensorData;        // Instantiate sensorData as a global


//  ---------------------------------------------------------------------------
//  getPingDist( void )
//  This routine returns the distance to the ultrasonic sensor in cm.
//  ---------------------------------------------------------------------------
unsigned int getPingDist( void )
{
  unsigned int pingTime;

  digitalWrite( PINGTRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite( PINGTRIGPIN, HIGH);
  delayMicroseconds(5);
  digitalWrite( PINGTRIGPIN, LOW);

  pingTime = pulseIn( PINGECHOPIN, HIGH );
  return pingTime /29 /2;
} // End of getPingDist routine


//  ===========================================================================
//  setup
//  ===========================================================================

void setup()
{
  pinMode( PINGTRIGPIN, OUTPUT );     // Set up ping trigger pin as output. (echo pin defaults to input)
  // pinMode( PINGECHOPIN, INPUT );   // Not needed as Arduino IO pins default to input

  sensorData.sensorID = SENSOR1;      // SENSOR1 defined in nrf_repeater_project.h

  analogReference( INTERNAL1V1 );     // Set up internal 1.1 V ADC reference.

  radio.begin();                      // Start the radio      
  radio.openWritingPipe(pipeSensor1); // Set write pipe
  radio.setPALevel(RF24_PA_LOW);      // Set radio output level
  radio.setDataRate( RF24_250KBPS );  // Set data rate
  radio.stopListening();              // Stop listening because this module only transmits

} // End of setup block


//  ======================================================================
//  main loop
//  This loop is repeated automatically.
//  ======================================================================

void loop()
{
  unsigned long startTick = ticks();  // To keep track of time for heartbeat
  unsigned long timePassed;           // Also to keep time, as above
  int pingDistCM = 1e4;               // Initialize to 1 km
  int temp, humid;                    // Hold temperature and humidity values
  float batV;                         // Used to calculate and hold this module battery voltage

  // Get battery voltage from this sensor module
  batV = 0;
  for( int x=0; x<5; x++ ) ;      // Let ADC settle
  for( int x=0; x<5; x++ )        // Take 5 readings to get the average
  {
    batV += analogRead( BATPIN );
  }
  batV /= (float)5;
  batV = (float)batV/230.0+0.4174;  // Compute actual battery voltage
  sensorData.sensorBatV = batV;     // Stuff battery voltage into radio packet.
  

  //  Main "do" loop to wait until either the sensor is read or until a
  //  timeout occures, at which point a heartbeat signal will be sent.
  do
  {
    delay( 100 );
    
    if( batV > BATLOWV )                // Read distance as long as the battery voltage is high enough.
      pingDistCM = getPingDist();
    else
      pingDistCM = 1e4;                  // Otherwise set the distance to infinity (as to not be detected)
    
    timePassed = ticks() - startTick;
  }
  while( (pingDistCM > PINGDIST ) && (timePassed < HBTICKSTO) ) ;   // Wait for detection from sensor or timeout

  dht11.readTemperatureHumidity( temp, humid );   // Read temperature and humidity and insert into data packet
  sensorData.sensorTemp = temp;
  sensorData.sensorHumid = humid;
  
  if( pingDistCM < PINGDIST )                          // Got a ping on the sensor
  {
    sensorData.dataType = PING | TEMP | HUMID;    // Send ping, temp, and humid data.
  }
  else
  {
    sensorData.dataType = HB | TEMP | HUMID;      // Send heartbeat, temp, and humid data.
  }

  if( batV < BATLOWV )                            // If battery voltage is low, then set battery low bit
    sensorData.dataType |= SBATLOW;
  radio.write(&sensorData, sizeof(sensorData));   // Send the data packet
  
  if( sensorData.dataType & PING )                // If detected presence, wait 5 seconds
    delay(10000);
}