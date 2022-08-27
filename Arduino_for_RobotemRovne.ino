/*
Arduino code for Robotem Rovne
*/

#include <DogGraphicDisplay.h>
#include "ubuntumono_b_16.h"
#include "dense_numbers_8.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "ifx007t.h"
#include <ArduinoNmeaParser.h>
#include <NavPoint.h>

void onRmcUpdate(nmea::RmcData const);
void onGgaUpdate(nmea::GgaData const);

Adafruit_ADS1015 ads1015;    // Construct an ads1015 at the default address: 0x48
Ifx007t mot1;
Ifx007t mot2;
ArduinoNmeaParser parser(onRmcUpdate, onGgaUpdate);
volatile unsigned int display_screen=0;
volatile bool nav_flag=0;
volatile float global_longitude=0.0;
volatile float global_latitude=0.0;
volatile float global_speed=0.0;
volatile float global_course=0.0;
volatile time_t global_timestamp=0;
volatile float gga_longitude=0.0;
volatile float gga_latitude=0.0;
volatile int gga_num_satellites=0;
volatile float gga_hdop=0.0;
volatile float gga_altitude=0.0;
volatile float gga_height=0.0;
volatile float gga_geoidal_separation=0.0;


//use these defines for Arduino MKR WIFI 1010
#define BACKLIGHTPIN A6
#define EMERGENCYSTOP A4
#define DIS_CS 6
#define DIS_A0 0
#define DIS_RESET 1
#define MOT1_1 2
#define MOT1_2 3
#define MOT1_EN A1
#define MOT2_1 4
#define MOT2_2 5
#define MOT2_EN A2

#define OFFSET 554
#define CUROFFSET1 112
#define CUROFFSET2 121

DogGraphicDisplay DOG;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float heading_soll=0;


void setup() {
  pinMode(EMERGENCYSTOP,  INPUT_PULLDOWN);   // set EMERGENCYSTOP pin to input with pull down
  pinMode(BACKLIGHTPIN,  OUTPUT);   // set backlight pin to output
  digitalWrite(BACKLIGHTPIN,  HIGH);  // enable backlight pin

  DOG.begin(DIS_CS,0,0,DIS_A0,DIS_RESET,DOGM128);   //CS = 15, 0,0= use Hardware SPI, A0 = 17, RESET = 16, EA DOGM128-5 (=128x64 dots)
  DOG.createCanvas(32, 32, 48, 4, 1);  // Canvas in buffered mode

  DOG.clear();  //clear whole display
  Wire.begin();
  DOG.string(0,0,UBUNTUMONO_B_16,"Scan I2C-Bus",ALIGN_CENTER);

//init serial ports
  Serial.begin(9600);
//  while (!Serial);             // Leonardo: wait for serial monitor
  Serial1.begin(9600);   // receiver u-blox NEO-7M NEO-6M
//  Serial1.begin(38400);   // receiver u-blox NEO-M8N

//init I2C
  Serial.println("\nI2C Scanner");
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(500);           // wait 0.5 seconds for next scan

  DOG.string(0,2,UBUNTUMONO_B_16,"Init BNO055",ALIGN_CENTER);
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  DOG.string(0,4,UBUNTUMONO_B_16,"Init ADS1015",ALIGN_CENTER);
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  ads1015.begin();
  mot1.begin(MOT1_1,MOT1_2,MOT1_EN);
  mot2.begin(MOT2_1,MOT2_2,MOT2_EN);

  DOG.clear();  //clear whole display
}

void loop() {
  static int counter=0;
  static int pwm=0;
  static int offset=DOG.display_width(); // static variable with the size of the display, so text starts at the right border
  int speed1, speed2, cur1, cur2;
  float heading_offset=0;

  int emergencystop;
  int16_t adc0, adc1, adc2, adc3, adc;
  float vin;
  char buffer[50];
  static NavPoint dest(48.995796, 12.837936);

//read sensor data
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Heading: ");
  Serial.println(orientationData.orientation.x);
  emergencystop=digitalRead(EMERGENCYSTOP);
// calculate motor voltage and current
  adc0 = ads1015.readADC_SingleEnded(0);
  adc1 = ads1015.readADC_SingleEnded(1);
  mot1.disable();
  mot2.disable();
  delay(1);
  adc2 = ads1015.readADC_SingleEnded(2);
  adc3 = ads1015.readADC_SingleEnded(3);
  adc = analogRead(A0);
  vin=(float)adc/1024.0*3.3*16;

  while (Serial1.available()) {
    int incomingByte=Serial1.read();
    parser.encode((char)incomingByte);
    Serial.write(incomingByte);   // read it and send it out Serial (USB)
  }


  if(counter<1000) counter++;
  if(emergencystop==0) 
  {
    counter=0;
    heading_soll=0;
  }
  if(counter<10)
  {
    mot1.stop();
    mot2.stop();
    pwm=50;
    heading_soll+=orientationData.orientation.x;
  }
  if(counter==10) 
  {
    heading_soll=heading_soll/10.0;
  }
  if(counter>10)
  {
//    pwm++;
    if(pwm<150) 
    {
      pwm+=5;
      mot1.pwm(pwm);
      mot2.pwm(pwm);
    }
    else
    {
      heading_offset=heading_soll-orientationData.orientation.x;
      if(heading_offset<-360.0) heading_offset+=360.0;
      if(heading_offset>360.0) heading_offset-=360.0;
      mot1.pwm(152-heading_offset*2.0);
      mot2.pwm(150+heading_offset*2.0);
    }
  }

  delay(100);  // wait a little bit

  speed1=adc2-OFFSET;
  if(speed1<0) speed1=-speed1;
  speed2=adc3-OFFSET;
  if(speed2<0) speed2=-speed2;

  cur1=adc0-CUROFFSET1;
  if(cur1<0) cur1=-cur1;
  cur2=adc1-CUROFFSET2;
  if(cur2<0) cur2=-cur2;

// print everything to serial out
  Serial.print("AIN: "); Serial.print(adc0);
  Serial.print(" "); Serial.print(adc1);
  Serial.print(" "); Serial.print(adc2);
  Serial.print(" "); Serial.print(adc3);
  Serial.print(" AIN : "); Serial.print(adc);
  Serial.print(" Vin : "); Serial.print(vin);
  Serial.print(" -*- "); Serial.print(speed1);
  Serial.print(" "); Serial.print(speed2);
  Serial.print(" -*- "); Serial.print(cur1);
  Serial.print(" "); Serial.print(cur2);
  Serial.println(" ");
// put all display output here.
  DOG.clear();

//display moving text
  DOG.string(offset,0,UBUNTUMONO_B_16,"Robotem Rovne!"); // print "Hello World" in line 3 at position offset
  offset--; // decrasye offset so text moves to the left
  if(offset<-132) offset=DOG.display_width(); //our text is 232 pixels wide so restart at this value

  DOG.string(0,2,DENSE_NUMBERS_8,itoa(orientationData.orientation.x,buffer,10),ALIGN_CENTER,STYLE_FULL);
  if(counter>10)
  {
    DOG.string(0,3,DENSE_NUMBERS_8,itoa(heading_soll,buffer,10),ALIGN_CENTER,STYLE_FULL);
    DOG.string(0,3,DENSE_NUMBERS_8,itoa((heading_offset*10),buffer,10),ALIGN_LEFT);
  }
  DOG.string(0,3,DENSE_NUMBERS_8,itoa(counter,buffer,10),ALIGN_RIGHT);
  DOG.string(0,2,DENSE_NUMBERS_8,itoa(emergencystop,buffer,10),ALIGN_LEFT);
  DOG.string(0,2,DENSE_NUMBERS_8,itoa(vin*100,buffer,10),ALIGN_RIGHT);

// draw circle with direction to target
  const int circle1_x=16;
  const int circle1_y=16;
  const int circle1_radius=15;
  float degree1=heading_offset;

  if(degree1<0.0) degree1+=360;

  float diff1_x=(circle1_radius-1)*sin(degree1*DEG_TO_RAD);
  float diff1_y=(circle1_radius-1)*cos(degree1*DEG_TO_RAD);

  DOG.clearCanvas();
  DOG.drawCircle(circle1_x, circle1_y, circle1_radius, false);
  DOG.drawArrow(circle1_x, circle1_y, circle1_x+diff1_x, circle1_y-diff1_y);
  DOG.flushCanvas();  

  delay(100);  // wait a little bit because code is not yet interrupt based
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onRmcUpdate(nmea::RmcData const rmc)
{
  char buf[30];

  time_t posix_timestamp = nmea::toPosixTimestamp(rmc.date, rmc.time_utc);
  if      (rmc.source == nmea::RmcSource::GPS)     Serial.print("GPS");
  else if (rmc.source == nmea::RmcSource::GLONASS) Serial.print("GLONASS");
  else if (rmc.source == nmea::RmcSource::Galileo) Serial.print("Galileo");
  else if (rmc.source == nmea::RmcSource::GNSS)    Serial.print("GNSS");

  Serial.print(" ");
  Serial.print(rmc.time_utc.hour);
  Serial.print(":");
  Serial.print(rmc.time_utc.minute);
  Serial.print(":");
  Serial.print(rmc.time_utc.second);
  Serial.print(".");
  Serial.println(rmc.time_utc.microsecond);
//  Serial.println(logger.timestamp_iso8601(posix_timestamp));
  if(rmc.time_utc.hour>=0)
  {
    global_timestamp=posix_timestamp;
    if((display_screen==0)||(display_screen==2))
    {
      sprintf(buf, "%02i:%02i:%02i",rmc.time_utc.hour,rmc.time_utc.minute,rmc.time_utc.second);
      DOG.string(0,2,UBUNTUMONO_B_16,buf); // print time in line 2 left
    }
    if((display_screen==2))
    {
      sprintf(buf, "%02i.%02i.%02i",rmc.date.day,rmc.date.month,rmc.date.year);
      DOG.string(70,2,DENSE_NUMBERS_8,buf); // print time in line 2 left
    }
  }

  if (rmc.is_valid)
  {
    global_longitude=rmc.longitude;
    global_latitude=rmc.latitude;
    global_speed=rmc.speed;
    global_course=rmc.course;
    nav_flag=1;
    Serial.print(" : LON ");
    Serial.print(rmc.longitude);
    Serial.print(" ° | LAT ");
    Serial.print(rmc.latitude);
    Serial.print(" ° | VEL ");
    Serial.print(rmc.speed);
    Serial.print(" m/s | HEADING ");
    Serial.print(rmc.course);
    Serial.print(" °");
    if(display_screen==0)
    {
      sprintf(buf, "%03.4f-%02.4f",rmc.longitude,rmc.latitude);
      DOG.string(0,0,UBUNTUMONO_B_16,buf); // print position in line 0 
      String speed(rmc.speed);
      DOG.string(70,2,DENSE_NUMBERS_8,speed.c_str()); // print speed in line 2 middle
      String speedkmh(rmc.speed*3.6);
      DOG.string(70,3,DENSE_NUMBERS_8,speedkmh.c_str()); // print speed in km/h in line 3 middle
      String course(rmc.course);
      DOG.string(100,2,DENSE_NUMBERS_8,course.c_str()); // print speed in line 2 right
    }
    if((display_screen==1)||(display_screen==2))
    {
      sprintf(buf, "%03.6f",rmc.longitude);
      DOG.string(0,0,DENSE_NUMBERS_8,buf); // print position in line 0 
      sprintf(buf, "%03.6f",rmc.latitude);
      DOG.string(0,1,DENSE_NUMBERS_8,buf); // print position in line 0 
    }
    if(display_screen==1)
    {
      sprintf(buf, "%03.2f",rmc.speed);
      DOG.string(70,0,DENSE_NUMBERS_8,buf); // print position in line 0 
      sprintf(buf, "%03.2f",rmc.speed*3.6);
      DOG.string(100,0,DENSE_NUMBERS_8,buf); // print position in line 0 
      sprintf(buf, "%03.2f",rmc.course);
      DOG.string(70,1,DENSE_NUMBERS_8,buf); // print position in line 0 
      
    }
  }
  else 
  {
    if(display_screen==0)
    {
      DOG.string(0,0,UBUNTUMONO_B_16,"data not valid  "); // print "not valid" in line 0 
      DOG.string(80,2,UBUNTUMONO_B_16,"    "); // print "    " in line 2 right 
    }
  }

  Serial.println();
}

void onGgaUpdate(nmea::GgaData const gga)
{
  Serial.print("GGA ");

  if      (gga.source == nmea::GgaSource::GPS)     Serial.print("GPS");
  else if (gga.source == nmea::GgaSource::GLONASS) Serial.print("GLONASS");
  else if (gga.source == nmea::GgaSource::Galileo) Serial.print("Galileo");
  else if (gga.source == nmea::GgaSource::GNSS)    Serial.print("GNSS");

  Serial.print(" ");
  Serial.print(gga.time_utc.hour);
  Serial.print(":");
  Serial.print(gga.time_utc.minute);
  Serial.print(":");
  Serial.print(gga.time_utc.second);
  Serial.print(".");
  Serial.print(gga.time_utc.microsecond);

  if (gga.fix_quality != nmea::FixQuality::Invalid)
  {
    gga_longitude=gga.longitude;
    gga_latitude=gga.latitude;
    gga_num_satellites=gga.num_satellites;
    gga_hdop=gga.hdop;
    gga_altitude=gga.altitude;
    gga_geoidal_separation=gga.geoidal_separation;
    gga_height=gga_altitude-gga_geoidal_separation;
    Serial.print(" : LON ");
    Serial.print(gga.longitude);
    Serial.print(" ° | LAT ");
    Serial.print(gga.latitude);
    Serial.print(" ° | Num Sat. ");
    Serial.print(gga.num_satellites);
    Serial.print(" | HDOP =  ");
    Serial.print(gga.hdop);
    Serial.print(" m | Altitude ");
    Serial.print(gga.altitude);
    Serial.print(" m | Geoidal Separation ");
    Serial.print(gga.geoidal_separation);
    Serial.print(" m");
  }

  Serial.println();
}
