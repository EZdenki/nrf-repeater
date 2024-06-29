/*
* nRF Repeater Project -- RECEIVER-2
*
* Hardware:
*   ATMEGA88 running at 8 MHz internal clock
*   DHT11 Temp/Humidity Sensor
*   Buzzer with transistor driver
*   nRF24L01+ Module
*   NOKIA 5110 LCD Display
*   LED1, LED2
*   BUT1, BUT2
*   RESET Button
*   Battery Voltage Monitor via divider: [VCC]--[8.2K]--[A0]--[2.2k]--[GND]
*   Serial Connection
*
*                          _____  _____
*                   RESET |1    \/   28| D19 - DHT11
*                     RXD |2         27| D18/A4 - NC
*                     TXD |3         26| D17 - BUZZER
*     NOKIA 5110 RST - D2 |4         25| D16 - BUT1
*     NOKIA 5110  CE - D3 |5         24| D15 - BUT2
*     NOKIA 5110  DC - D4 |6         23| A0  - BATTERY
*                     VCC |7         22| GND
*                     GND |8         21| AREF
*              XTAL1 - NC |9         20| AVCC
*              XTAL2 - NC |10        19| SCK  - nRF / ISP
*     NOKIA 5110 DIN - D5 |11        18| MISO - nRF / ISP
*     NOKIA 5110 CLK - D6 |12        17| MOSI - nRF / ISP
*               LED1 - D7 |13        16| D10  - nRF CE
*               LED2 - D8 |14        15| D9   - nRF CSN
*                         |____________|
*                            ATMEGA88
*/

#include <nrf_repeater_project.h>
#include <DHT11.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <NOKIA5110_TEXT.h>

//  -------------------------------------
//  DHT11 Setup
#define DHT11PIN 19
DHT11 dht11( DHT11PIN );            // Declare DHT11 temp sensor instance

//  -------------------------------------
//  Buzzer Setup
#define BUZPIN 17

//  -------------------------------------
//  Nokia 5110 LCD Setup
// LCD Nokia 5110 pinout left to right
// RST / CE / DC / DIN / CLK / VCC /LIGHT / GND
#define RST 2
#define CE 3
#define DC 4
#define DIN 5
#define CLK 6
#define BL 21

// Create an LCD object and set LCD Constants
NOKIA5110_TEXT mylcd(RST, CE, DC, DIN, CLK);

#define inverse  false  // set to true to invert display pixel color
#define contrast 0xb7   // default is 0xBF set in LCDinit,
                        // Try 0xB1 <-> 0xBF if your display is too dark/dim
#define bias 0x14       // LCD bias mode 1:48: Try 0x12 or 0x13 or 0x14

//  -------------------------------------
//  nRF24L01+ Radio Setup
#define NRFCEPIN 10
#define NRFCSNPIN 9

RF24 radio( NRFCEPIN, NRFCSNPIN ); // CE, CSN

//  -------------------------------------
//  LED, Battery monitor, Button Setup
#define LED1PIN 7     // LED 1 Pin
#define LED2PIN 8     // LED 2 Pin
#define LED3PIN 20    // LED 3 Pin
#define BUT1PIN 16    // Button 1 Pin
#define BUT2PIN 15    // Button 1 Pin
#define BATPIN  A0    // Battery Voltage Monitor Pin

#define LOWBATV 2.5   // Low-voltage level to trigger battery-change warning.

//  tone2
//  Generate a generic beeping tone as an alert
void tone2( void )
{
  tone( BUZPIN, 1000, 500 );
  delay( 500 );
  tone( BUZPIN, 800, 750 );
}
//  tone1
//  Generate a generic beeping tone as an alert
void tone1( void )
{
  tone( BUZPIN, 784, 150 );
  delay( 200 );
  tone( BUZPIN, 987, 200 );
  delay( 200 );
  tone( BUZPIN, 880, 200 );
 
}

//  Decalare Global Variables

SensorStruct_t sensorData;    // Declare structure to hold received data
int pingCount1 = 0;           // Holds current ping count for sensor 1
int pingCount2 = 0;           // Holds current ping count for sensor 1
int BLState = LOW;            // Holds backlight state (LOW = backlight OFF, HIGH = backlight ON)
int LCDState = LOW;           // Holds current LCD display state (LOW = Normal, HIGH = BAT V display)

float lastRepeaterBatV;
float lastSensor1BatV;
float lastSensor2BatV;

// Variable to hold uC Tick counter to detect for timeouts
extern volatile unsigned long timer0_overflow_count;


void setup() {
  pinMode( LED1PIN, OUTPUT );       // LED 1 Pin (for SENSOR1) as output
  pinMode( LED2PIN, OUTPUT );       // LED 2 Pin (for SENSOR2) as output
  pinMode( LED3PIN, OUTPUT );       // LED 3 Pin (for Warning) as output
  pinMode( DHT11PIN, INPUT );       // DHT11 Sensor Pin as input
  pinMode( BUT1PIN, INPUT_PULLUP ); // Button 1 as input with internal pullup
  pinMode( BUT2PIN, INPUT_PULLUP ); // Button 2 as input with internal pullup
  pinMode( BL, OUTPUT );            // Pin to drive LCD backlight
  
  analogReference( INTERNAL1V1 );   // Set analog reference to internal 1.1V

  
  //  Set up LCD
  delay(50);
  mylcd.LCDInit(inverse, contrast, bias); // init  the lCD
  mylcd.LCDClear(0x00); // Clear whole screen
  mylcd.LCDFont(LCDFont_Default); // Set the font
  mylcd.LCDgotoXY(0, 0); // (go to (X , Y) (0-84 columns, 0-5 blocks) top left corner

  //  Set up Radio
  radio.begin();
  radio.openReadingPipe( 1, pipeRepeater1 );
  radio.openReadingPipe( 2, pipeSensor1 );
  radio.setPALevel( RF24_PA_LOW );
  radio.setDataRate( RF24_250KBPS );
  radio.startListening();

  tone1();    // Give an initial beep to let user know the program is starting
} 


//  ==========================================================================
//  loop
//  ==========================================================================
void loop() {
  int temp, humid;
  float batV;

  //  ------------------------------------------------------------------------
  //  Get Local Temp and Humidity Readings from DHT11 Sensor
  //  ------------------------------------------------------------------------
  dht11.readTemperatureHumidity( temp, humid );
  mylcd.LCDgotoXY( 0, 0 );
  mylcd.print( "Lo ");

  batV = 0;                   // Start by getting local battery voltage
  for( int x=0; x<5; x++ )    // Get some analog readings to stabilize
    analogRead( BATPIN );
  for( int x=0; x<5; x++ )    // Get average of 5 readings
    batV += analogRead( BATPIN );
  batV /= 5;
  batV = batV / 211.33 + 0.1372;  // Calibration based on test results

  if( !LCDState )
  {
    mylcd.print( temp );
    mylcd.print( "C " );
    mylcd.print( humid );
    mylcd.print( "%  " );
  }
  else
  {
    mylcd.print( batV );
    mylcd.print( " V" );
  }

  mylcd.LCDgotoXY( 11*7, 0 );     // Display the heartbeat or battery voltage mark
  if( batV < LOWBATV )
  {
    digitalWrite( LED3PIN, LOW );
    mylcd.print( " " );
    delay( 500 );
    mylcd.LCDgotoXY( 11*7, 0 );
    mylcd.print( "B");
    digitalWrite( LED3PIN, HIGH );
  }
  else
  {
    mylcd.print( "*" );
    delay( 500 );
    mylcd.LCDgotoXY( 11*7, 0 );
    mylcd.print( " " );
  }

  if( !digitalRead( BUT1PIN ) )
  {
    BLState = ~BLState;
    digitalWrite( BL, BLState );
    delay(1000);
    while( !digitalRead( BUT1PIN )) ;
  }

  if( !digitalRead( BUT2PIN ) )
  {
    LCDState = ~LCDState;
    mylcd.LCDClear();
    if( LCDState )
    {
      mylcd.LCDgotoXY( 0, 1 );
      mylcd.print( "FD " );
      mylcd.print( lastSensor1BatV );
      mylcd.print( " V" );
      mylcd.LCDgotoXY( 0, 2 );
      mylcd.print( "CR " );
      mylcd.print( lastSensor2BatV );
      mylcd.print( " V" );
      mylcd.LCDgotoXY( 0, 5 );
      mylcd.print( "R  " );
      mylcd.print( lastRepeaterBatV );
      mylcd.print( " V" );
    }
    
    while( !digitalRead( BUT2PIN ) ) ;
  }

  //  ========================================================================
  //  Check Radio for incoming signal
  //  ========================================================================
  if (radio.available()) {    //  Check radio for data and read in data if available

    radio.read(&sensorData, sizeof(sensorData));    // Get sensor data packet

    if( sensorData.repeaterBatV != 0 )
    {
      mylcd.LCDgotoXY( 0, 5 );                      // Display repeater info
      mylcd.print( "R " );
      mylcd.LCDgotoXY( 1*7, 5 );
      lastRepeaterBatV = sensorData.repeaterBatV;  // Hold repeater battery voltage
    }

    if( sensorData.dataType & RBATLOW )
    {
      digitalWrite( LED3PIN, LOW );        // Turn on LED3
      delay( 1000 );                        // If the repeater battery voltage is low,
      mylcd.print( "B" );                   // the flash on the "B" sign.
      digitalWrite( LED3PIN, HIGH );        // Turn on LED3
    }
    else
    {
      mylcd.print( "*" );
      delay( 500 );
      mylcd.LCDgotoXY( 1*7, 5 );
      mylcd.print( " " );
    }
    
    if( LCDState && (lastRepeaterBatV != 0) )   // Display the repeater battery voltage if this
    {                                           // packet came from a repeater. If the repeater
      mylcd.print( " " );                       // battery voltage is zero, then the packet did
      mylcd.print( lastRepeaterBatV );          // not come from the repeater, and should not be
      mylcd.print( " V" );                      // displayed.
    }

    //  ------------------------------------------------------------------------
    // Read Sensor 1 (Front Door)
    //  ------------------------------------------------------------------------
    if( sensorData.sensorID == SENSOR1 )
    {
      lastSensor1BatV = sensorData.sensorBatV;    // Hold battery voltage for Sensor 1

      mylcd.LCDgotoXY(0, 1);                      // Display front door (FD) tag
      mylcd.LCDString("FD ");

      if( !LCDState )                             // In normal state, display temp/humidity
      {
        mylcd.print( (int)sensorData.sensorTemp );
        mylcd.print( "C " );
        mylcd.print( (int)sensorData.sensorHumid );
        mylcd.print( "%  " );

      }
      else                                          // Otherwise, display battery voltages for the sensor
      {
        mylcd.print( lastSensor1BatV );
        mylcd.print( " V " );
      }

      if( sensorData.dataType & PING  )   // If got a ping from the sensor module...
      {
        pingCount1++;                     // Increase ping counter, but stop at 9
        if( pingCount1 > 9 )
          pingCount1 = 9;
        tone1();
      }

      if( pingCount1 )                    // If there is a ping count, then flash it as normally
      {
        mylcd.LCDgotoXY( 11*7, 1 );       // ON to indicate a heartbeat.
        mylcd.print( " " );
        delay( 500 );
        mylcd.LCDgotoXY( 11*7, 1 );
        mylcd.print( pingCount1 );
        digitalWrite( LED1PIN, LOW );
        delay( 100 );
        digitalWrite( LED1PIN, HIGH );
        delay( 100 );
      }
      else
      {                                 // If no ping count, then flash on a "*" to signal heartbeat.
        mylcd.LCDgotoXY( 11*7, 1 );
        mylcd.print( "*" );
        delay( 500 );
        mylcd.LCDgotoXY( 11*7, 1 );
        mylcd.print( " " );
      }

      if( sensorData.dataType & SBATLOW )  // If the sensor module battery voltage is low,
      {
        digitalWrite( LED3PIN, LOW );      // Flash ON LED3 
        mylcd.LCDgotoXY( 11*7, 1 );        // and flash on the "B" signal
        mylcd.print( " " );
        delay( 500 );
        mylcd.LCDgotoXY( 11*7, 1 );
        mylcd.print( "B" );
        digitalWrite( LED3PIN, HIGH );
      }

    }

    //  ------------------------------------------------------------------------
    //  Read Sensor 2 (Classroom)
    //  ------------------------------------------------------------------------
    else if( sensorData.sensorID == SENSOR2 )
    {
      lastSensor2BatV = sensorData.sensorBatV;

      mylcd.LCDgotoXY(0, 2);
      mylcd.LCDString("CR ");

      if( !LCDState )
      {
        mylcd.print( (int)sensorData.sensorTemp );
        mylcd.print( "C " );
        mylcd.print( (int)sensorData.sensorHumid );
        mylcd.print( "%  " );
      }
      else
      {
        mylcd.print( lastSensor2BatV );
        mylcd.print( " V " );
      }

      if( sensorData.dataType & PING )
      {
        pingCount2++;
        if ( pingCount2 > 9 )
          pingCount2 = 9;
        tone1();
      }
      if( pingCount2 )
      {
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( " " );
        delay( 500 );
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( pingCount2 );
        for( int x=0; x<pingCount2; x++ );
        {
          digitalWrite( LED2PIN, LOW );
          delay( 100 );
          digitalWrite( LED2PIN, HIGH );
          delay( 100 );
        }
      }
      else
      {
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( "*" );
        delay( 500 );
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( " " );
      }

      if( sensorData.dataType & SBATLOW )
      {
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( " " );
        delay( 100 );
        mylcd.LCDgotoXY( 11*7, 2 );
        mylcd.print( "B" );
        digitalWrite( LED3PIN, HIGH );
      }

    }
  }
}