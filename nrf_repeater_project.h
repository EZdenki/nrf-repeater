// nrf_repeater_project Header file

const uint64_t pipeSensor1   = 0xF0F0F0F0A1; // Sensor 1 Pipe
const uint64_t pipeSensor2   = 0xF0F0F0F0A2; // Sensor 2 Pipe
const uint64_t pipeSensor3   = 0xF0F0F0F0A3; // Sensor 3 Pipe
const uint64_t pipeSensor4   = 0xF0F0F0F0A4; // Sensor 4 Pipe
const uint64_t pipeRepeater1 = 0xF0F0F0F0A5; // Repeater 1 Pipe
const uint64_t pipeRepeater2 = 0xF0F0F0F0AF; // Repeater 2 Pipe


#define  HB       (1<<0)
#define  PING     (1<<1)
#define  TEMP     (1<<2)
#define  HUMID    (1<<3)
#define  SBATLOW  (1<<4)
#define  RBATLOW  (1<<5)

typedef enum
{
  REPEATER1, REPEATER2, REPEATER3, REPEATER4,
  SENSOR1,   SENSOR2,   SENSOR3,   SENSOR4,
  SENSOR5,   SENSOR6,   SENSOR7,   SENSOR8,
  SENSOR9,   SENSOR10,  SENSOR11,  SENSOR12,
  SENSOR13,  SENSOR14,  SENSOR15,  SENSOR16
} SensorID;

typedef struct
{
  int   sensorID;
  int   dataType;
  float sensorTemp;
  float sensorHumid;
  float sensorBatV;
  float repeaterBatV;
} SensorStruct_t;


// #define SensorHBTicksTO  5000    // 30000 ticks = 1 min until timeout
				    // However, this should be defined for each sensor module

extern volatile unsigned long timer0_overflow_count;
unsigned long ticks()
{
  return timer0_overflow_count;
}
