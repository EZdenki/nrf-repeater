/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
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
#include <toneAC.h>
#include <NOKIA5110_TEXT.h>

DHT11 dht11( 19 );            // Declare DHT11 temp sensor instance

SensorStruct_t sensorData;    // Declare structure to hold received data

// LCD Nokia 5110 pinout left to right
// RST / CE / DC / DIN / CLK / VCC /LIGHT / GND
#define RST 0
#define CE 1
#define DC 2
#define DIN 3
#define CLK 4

// Create an LCD object
NOKIA5110_TEXT mylcd(RST, CE, DC, DIN, CLK);

#define inverse  false // set to true to invert display pixel color
#define contrast 0xb7 // default is 0xBF set in LCDinit, Try 0xB1 <-> 0xBF if your display is too dark/dim
#define bias 0x13 // LCD bias mode 1:48: Try 0x12 or 0x13 or 0x14

#define BACKLIGHT 6

void
flashLCD( int times, int duration )
{
  for( int x=0; x<times; x++ )
  {
    digitalWrite( BACKLIGHT, LOW );
    delay( duration );
    digitalWrite( BACKLIGHT, HIGH );
    delay( duration );
  }
}
void tone1( void )
{
  
  //for(int x=0; x<8; x++)
  {
    for( int tone=400; tone<1000; tone ++)
    {
      toneAC(tone);
      //delay(2);
    }
        for( int tone=1000; tone>400; tone --)
    {
      toneAC(tone);
      //delay(2);
    }
    toneAC(0);
  }
}

extern volatile unsigned long timer0_overflow_count;


RF24 radio(7, 8); // CE, CSN
#define BUZZER 9
#define BACKLIGHT 6
#define PINGLED 5
#define BATPIN A4


int radCnt = 0;
int pingCount = 0;

void setup() {
  
  pinMode( BACKLIGHT, OUTPUT );
  digitalWrite( BACKLIGHT, HIGH );

  pinMode( PINGLED, OUTPUT );
  pinMode( 19, INPUT );
  analogReference( INTERNAL1V1 );
  
  delay(50);
  mylcd.LCDInit(inverse, contrast, bias); // init  the lCD
  mylcd.LCDClear(0x00); // Clear whole screen
  mylcd.LCDFont(LCDFont_Default); // Set the font
  mylcd.LCDgotoXY(0, 0); // (go to (X , Y) (0-84 columns, 0-5 blocks) top left corner
  //mylcd.LCDgotoXY(0, 2);
  //mylcd.LCDString("Ping Count:");
  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipeReceiver);
  radio.setPALevel( RF24_PA_LOW );
  radio.setDataRate( RF24_250KBPS );
  radio.startListening();
  tone1();
} 

void loop() {
  int temp, humid;
  float batV;

  batV = 0;
  for( int x=0; x<100; x++ )
  {
    batV += analogRead( BATPIN );
  }
  batV /= 100;
//  batV = analogRead( BATPIN );
  mylcd.LCDgotoXY(0, 1 );
  mylcd.print("Bat:");
  mylcd.print( (int)batV );
  mylcd.print( " " );
  sensorData.batV = (float)batV/230.0+0.4174;
  mylcd.print(" ");
  mylcd.print( sensorData.batV );  

  dht11.readTemperatureHumidity( temp, humid );
  sensorData.sensorTemp = temp;
  sensorData.sensorHumid = humid;

  mylcd.LCDgotoXY( 0, 0 );
  mylcd.print( "Loc: ");
  mylcd.print( temp );
  mylcd.print( "C " );
  mylcd.print( humid );
  mylcd.print( "%" );

  if (radio.available()) {
    radio.read(&sensorData, sizeof(sensorData));

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
        pingCount++;
        for( int x=0; x<pingCount; x++)
          tone1();
        mylcd.print( pingCount );
        flashLCD( pingCount, 200 );
      }
      if( pingCount )
        for( int x=0; x<pingCount; x++ )
        {
          digitalWrite( PINGLED, LOW );
          delay( 200 );
          digitalWrite( PINGLED, HIGH );
          delay( 200 );
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
        pingCount++;
        for( int x=0; x<pingCount; x++)
          tone1();
        mylcd.print( pingCount );
      }
      if( pingCount )
        for( int x=0; x<pingCount; x++ )
        {
          digitalWrite( PINGLED, LOW );
          delay( 200 );
          digitalWrite( PINGLED, HIGH );
          delay( 200 );
        }
    }
  }
}