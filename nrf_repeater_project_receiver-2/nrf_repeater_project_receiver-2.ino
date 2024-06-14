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
#define BUT1PIN 16    // Button 1 Pin
#define BUT2PIN 15    // Button 1 Pin
#define BATPIN  A0    // Battery Voltage Monitor Pin



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

// Variable to hold uC Tick counter to detect for timeouts
extern volatile unsigned long timer0_overflow_count;


void setup() {
  pinMode( LED1PIN, OUTPUT );       // LED 1 Pin as output
  pinMode( LED2PIN, OUTPUT );       // LED 2 Pin as output
  pinMode( DHT11PIN, INPUT );       // DHT11 Sensor Pin as input
  analogReference( INTERNAL1V1 );   // Set analog reference to internal 1.1V

  
  //  Set up LCD
  delay(50);
  mylcd.LCDInit(inverse, contrast, bias); // init  the lCD
  mylcd.LCDClear(0x00); // Clear whole screen
  mylcd.LCDFont(LCDFont_Default); // Set the font
  mylcd.LCDgotoXY(0, 0); // (go to (X , Y) (0-84 columns, 0-5 blocks) top left corner

  //  Set up Radio
  radio.begin();
  radio.openReadingPipe(1, pipeReceiver);
  radio.setPALevel( RF24_PA_LOW );
  radio.setDataRate( RF24_250KBPS );
  radio.startListening();

  tone1();    // Give an initial beep to let user know the program is starting
} 

void loop() {
  int temp, humid;
  float batV;

  //  Get temp and humidity readings from DHT11 sensor
  dht11.readTemperatureHumidity( temp, humid );

  mylcd.LCDgotoXY( 0, 0 );
  mylcd.print( "Loc: ");
  mylcd.print( temp );
  mylcd.print( "C " );
  mylcd.print( humid );
  mylcd.print( "%" );

  batV = 0;                   // Start by getting local battery voltage
  for( int x=0; x<5; x++ )    // Get some analog readings to stabilize
    analogRead( BATPIN );
  for( int x=0; x<5; x++ )    // Get average of 5 readings
    batV += analogRead( BATPIN );
  batV /= 5;

  mylcd.LCDgotoXY(0, 1 );     // Display the battery voltage
  batV = batV / 211.33 + 0.1372;  // Calibration based on test results
  mylcd.print( batV );
  if (radio.available()) {    //  Check radio for data and read in data if available
    radio.read(&sensorData, sizeof(sensorData));
    mylcd.print( " R:" );
    mylcd.print( sensorData.repeaterBatV );

    // ----------------------------------------
    // Read Sensor 1 (Classroom)
    // ----------------------------------------
    if( sensorData.sensorID == SENSOR1 )
    {

      mylcd.LCDgotoXY(0, 2);
      mylcd.LCDString("FD:  ");
      mylcd.print( (int)sensorData.sensorTemp );
      mylcd.print( "C " );
      mylcd.print( (int)sensorData.sensorHumid );
      mylcd.print( "% " );

      mylcd.LCDgotoXY( 0, 3 );
      if( sensorData.dataType & HB )
      {
        mylcd.print( "    " );
        delay( 300 );
        mylcd.LCDgotoXY( 0, 3 );
        //mylcd.print( "HB " );
        mylcd.print( sensorData.batV );
      }
      else if( sensorData.dataType & PING )
      {
        mylcd.print( "PING  PC:" );
        pingCount1++;
        mylcd.print( pingCount1 );
        tone1();
      }
      if( pingCount1 )
        for( int x=0; x<pingCount1; x++ )
        {
          digitalWrite( LED1PIN, LOW );
          delay( 100 );
          digitalWrite( LED1PIN, HIGH );
          delay( 100 );
        }
    }
    else if( sensorData.sensorID == SENSOR2 )
    {
      mylcd.LCDgotoXY(0, 4);
      mylcd.LCDString("CR:  ");
      mylcd.print( (int)sensorData.sensorTemp );
      mylcd.print( "C " );
      mylcd.print( (int)sensorData.sensorHumid );
      mylcd.print( "% " );

      mylcd.LCDgotoXY( 0, 5 );
      if( sensorData.dataType & HB )
      {
        mylcd.print( "    " );
        delay( 300 );
        mylcd.LCDgotoXY( 0, 5 );
        //mylcd.print( "HB " );
        mylcd.print( sensorData.batV );
      }
      else if( sensorData.dataType & PING )
      {
        mylcd.print( "PING  PC:" );
        pingCount2++;
        mylcd.print( pingCount2 );
        tone1();
      }
      if( pingCount2 )
        for( int x=0; x<pingCount2; x++ )
        {
          digitalWrite( LED2PIN, LOW );
          delay( 100 );
          digitalWrite( LED2PIN, HIGH );
          delay( 100 );
        }
    }
  }
}