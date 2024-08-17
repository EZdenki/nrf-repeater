//  nRF Repeater Project -- Repeater Module
//
//  A simple repeater that receives data packets from sensor transmitter modules and
//  relays the data packets to the main receiver module (or, potentially later, another
//  repeater down the line.) Will send a data packet as soon as it it received. Will also
//  include the battery voltage of the repeater module and will set the RBATLOW bit of
//  dataType in case the battery voltage is getting low. In case there are no packets to
//  relay, it will periodically send a heartbeat signal that includes the battery voltage
//  information.
//
//  Based on:
//  Arduino Wireless Communication Tutorial
//  by Dejan Nedelkovski, www.HowToMechatronics.com
//
//  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
//
//

#include <nrf_repeater_project.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//  Define IO pins for this project
#define ACTLEDPIN 8     // Activity LED on D8/P14
#define BATPIN    A0    // Battery voltage divider output on A0/D14/P23
#define NRFCEPIN  10    // nRF radio CE pin on D10/P16
#define NRFCSNPIN 9     // nRF radio CSN pin on D9/P15

//  Define other constants
#define HBTICKSTO 15000 // Ticks until timout and send heartbeat. 15,000 ticks = ~30 seconds
#define BATLOWV   2.5   // Indicates the lower voltage before setting the RBATLOW warning bit

//  Instantiate radio object and data packet variables
RF24 radio( NRFCEPIN, NRFCSNPIN ); // CE, CSN
SensorStruct_t sensorData;


//  ===========================================================================
//  setup
//  ===========================================================================
void setup()
{
  //  Set up hardware
  analogReference( INTERNAL1V1 );         // Set analog reference to internal 1.1V
  sensorData.sensorID = REPEATER1;        // Set the ID of this module. (REPEATER1 defined in nrf_repeater_project.h)
  pinMode( ACTLEDPIN, OUTPUT);

  radio.begin();                            // Start the radio
  radio.openReadingPipe( 1, pipeSensor2 );  // Open and set reading pipe for sensor module 2 (Classroom)
  radio.openWritingPipe( pipeRepeater1 );   // Set pipe of who this module is
  radio.setPALevel( RF24_PA_MAX );          // Set the radio output level
  radio.setDataRate( RF24_250KBPS );        // Set data rate

  for( int x=0; x<5; x++ )                  // Flash the activity LED to indicate startup
  {
    digitalWrite( ACTLEDPIN, HIGH );
    delay(100);
    digitalWrite(ACTLEDPIN, LOW);
    delay(100);
  }
} // End of setup block


//  ===========================================================================
//  loop
//  ===========================================================================

void loop()
{
  unsigned long startTick = ticks();  // To keep track of time for heartbeat
  unsigned long timePassed;           // Also to keep time, as above
  float batV;                         // Will hold battery voltage of repeater
  int nrfGet;                         // Indicates if had gotten data packet from a sensor module

  delay( 5 );
  radio.startListening();             // Put radio into listening mode
  delay( 5 );

  //  Wait here until either the a data packet is recieved or until a certain time has passed.
  do
  { 
    nrfGet = radio.available();
    timePassed = ticks() - startTick;
  } while ( !nrfGet && ( timePassed < HBTICKSTO ) ) ;

  if( nrfGet )                      // If received a data packet, then download the packet
    radio.read( &sensorData, sizeof( sensorData ) );  // Download the packet
  else
  {
    sensorData.dataType = 0;          // Didn't receive a packet, but need to send a heartbeat from the
    sensorData.sensorID = REPEATER1;  // repeater, so set up a packet to identify the repeater and show
    sensorData.dataType = HB;         // this as a heartbeat from the repeater.
  }

  batV = 0;                         // Get the local battery voltage for the repeater
  for( int x=0; x<5; x++ )          // Get some analog readings to stabilize
    analogRead( BATPIN );
  for( int x=0; x<5; x++ )          // Get average of 5 readings
    batV += analogRead( BATPIN );
  batV /= 5;

  batV = batV /210.0 + 0.12381;      // Convert ADC value to actual voltage

  sensorData.repeaterBatV = batV;
  if( batV < BATLOWV )                // If the repeater battery voltage is low, then
    sensorData.dataType |= RBATLOW;   // set the remote battery low bit in the datapacket

  delay( 10 );                        // Blink the activity LED to indicate a transmission
  digitalWrite( ACTLEDPIN, HIGH );
  delay( 10 );
  digitalWrite( ACTLEDPIN, LOW );

  radio.stopListening();              // Turn off listening and then send the data packet.
  radio.write( &sensorData, sizeof( sensorData ) );
} // End of loop block