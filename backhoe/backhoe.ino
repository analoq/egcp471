#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define BOOM_1    2
#define BOOM_2    3
#define STICK_1   4
#define STICK_2   5
#define BUCKET_1  6
#define BUCKET_2  7
#define HOUSE_1   8
#define HOUSE_2   9

Adafruit_MMA8451 stick_mma = Adafruit_MMA8451();
Adafruit_MMA8451 boom_mma = Adafruit_MMA8451();

void setup()
{
  Serial.begin(9600);
  while(!Serial)
  {
      delay(10);
  }
  
  pinMode(BOOM_1, OUTPUT);
  pinMode(BOOM_2, OUTPUT);
  pinMode(STICK_1, OUTPUT);
  pinMode(STICK_2, OUTPUT);
  pinMode(BUCKET_1, OUTPUT);
  pinMode(BUCKET_2, OUTPUT);
  pinMode(HOUSE_1, OUTPUT);
  pinMode(HOUSE_2, OUTPUT);

  if (!boom_mma.begin(0x1C))  // use alternate address
  {
    Serial.println("Couldnt start boom accelerometer");
    while (1);
  }

  if (!stick_mma.begin(0x1D))  // use default address
  {
    Serial.println("Couldnt start stick accelerometer");
    while (1);
  }
}

void fire(unsigned char port1, unsigned char port2, unsigned char dir, unsigned char mag)
{
  if (dir)
  {
    analogWrite(port1, mag);
    analogWrite(port2, 0);
  }
  else
  {
    analogWrite(port2, mag);
    analogWrite(port1, 0);
  }
}

sensors_event_t sensor_boom()
{
  /* Get a new sensor event */
  sensors_event_t event; 
  boom_mma.getEvent(&event);
  return event;
}

sensors_event_t sensor_stick()
{
  /* Get a new sensor event */
  sensors_event_t event; 
  stick_mma.getEvent(&event);
  return event;
}

void loop()
{
  unsigned char cmd[3];
  unsigned char bytes_read = Serial.readBytes(cmd, 3);
  sensors_event_t event;

  if ( bytes_read )
  {
    switch ( cmd[0] )
    {
      case 0x00:
        Serial.println("OK");
        break;
      case 0x01:
        fire(BOOM_1, BOOM_2, cmd[1], cmd[2]);
        break;
      case 0x02:
        fire(STICK_1, STICK_2, cmd[1], cmd[2]);
        break;
      case 0x03:
        fire(BUCKET_1, BUCKET_2, cmd[1], cmd[2]); 
        break;
      case 0x04:
        fire(HOUSE_1, HOUSE_2, cmd[1], cmd[2]);
        break;
      case 0x11:
        event = sensor_boom();
        Serial.println(event.acceleration.y, 10);
        break;
      case 0x12:
        event = sensor_stick();
        Serial.println(event.acceleration.y, 10);
        break;
    }
  }

  delay(100);
}
