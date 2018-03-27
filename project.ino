#include <SoftwareSerial.h>


int forcecounter = 0;
int state = 0;
char pitoarduino;
String messagetopi;
char check;
float lat = 12.843589;
float lon = 80.153384;
const int AOUTpin=6;//the AOUT pin of the alcohol sensor goes into analog pin A0 of the arduino
const int DOUTpin=51;//the DOUT pin of the alcohol sensor goes into digital pin D8 of the arduino
const int ledPinAWL=30;//the anode of the LED connects to digital pin D13 of the arduino
const int ledPinAWH=31;
const int ledPinALL=33;
const int ledPinALH=32;
int alcoholread;
//gps variables and function

SoftwareSerial mySerial(8, 9);
//TinyGPS gps;
//
//void gpsdum0p(TinyGPS &gps);


byte cardirection;//direction motion


int lockdown = 0;


int ml1 = 40;
int ml2 = 42;
int mr1 = 48;
int mr2 = 46; //for the motor pins

int accidentpressurePin = A3; // reading the analog input of the pressure sensor
int accidentforce = 0; // the value from the first foece sensor
int driverpressurePin = A4;
int driverforce;



int vx, vy, vz, sum; //accelerometer
int x = A0;
int y = A1;
int z = A2;
int i = 0;
int incper = 3;
int decper = 3;

int orientation = 0;

float xsum = 0, ysum = 0, zsum = 0;




void updatevalues()
{
  vx = analogRead(x);
  vy = analogRead(y);
  vz = analogRead(z);
}

void forward()
{
  //Serial.println(" FWRD ");
  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);
  digitalWrite(mr1, HIGH);
  digitalWrite(mr2, LOW);
}

void stand()
{
  //Serial.println(" Stand ");
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, LOW);
  digitalWrite(ml1, LOW);
  digitalWrite(ml2, LOW);
}


//gps functions
void stall()
{
  while (1)
  {
    stand();
  }
}

void forcefunction()
{
  for (int i = 0; i < 250; i++)
  {
    accidentforce = analogRead(accidentpressurePin);
    //Serial.print("hello");
  }
}

void setup()
{
Serial.begin(9600);

  //motor
  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);
  pinMode(mr1, OUTPUT);
  pinMode(mr2, OUTPUT);

 //alcohol sensor
 pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
pinMode(ledPinAWL, OUTPUT);
pinMode(ledPinAWH, OUTPUT);
pinMode(ledPinALL, OUTPUT);
pinMode(ledPinALH, OUTPUT);


  //accelerometer
  pinMode(x, INPUT);
  pinMode(y, INPUT);
  pinMode(z, INPUT);

  delay(1000);
  updatevalues();
  updatevalues();
  updatevalues();
  xsum = vx;
  ysum = vy;
  zsum = vz;
  for (i = 0; i < 200; i++)
  {
    updatevalues();
    xsum = (xsum + vx) / 2;
    ysum = (ysum + vy) / 2;
    zsum = (zsum + vz) / 2;
    delay(10);
  }

  //Serial.print("Sizeof(gpsobject) = ");
  //Serial.println(sizeof(TinyGPS));

  delay(1000);
  //Serial.begin(115200);
}


void loop()
{

  if (forcecounter == 0 )
  {
    forcefunction();
    forcecounter++;
    forward();
  }
  accidentforce = analogRead(accidentpressurePin);
  String force = String(accidentforce);
  alcoholread = analogRead(AOUTpin);
  delay(100);
  //Serial.println(accidentforce);
   messagetopi = force + ";" + lat + ";" + lon + ";" + orientation+";"+alcoholread+";END";
  Serial.println(messagetopi);
  
  if (alcoholread >= 100 && alcoholrea
  d <300){
  digitalWrite(ledPinAWH, HIGH);
  digitalWrite(ledPinAWL, LOW);
   digitalWrite(ledPinALH, LOW);
  digitalWrite(ledPinALL, LOW);
 
  }
  else if (alcoholread >= 300){

  digitalWrite(ledPinALH, HIGH);
  digitalWrite(ledPinALL, LOW);
  }
  else{
     digitalWrite(ledPinAWH, LOW);
  digitalWrite(ledPinAWL, LOW);
   digitalWrite(ledPinALH, LOW);
  digitalWrite(ledPinALL, LOW);
  }
 // int alcolevel = analogRead(alcoholAnalog);
  //int alcolim = digitalRead(alcoholDigital);
  //Serial.println(alcolevel);
  //Serial.println(alcolim);
  //car car collision
//  if (accidentforce > 1000)
//  {
//    state = 1;
//    messagetopi = force + ";" + lat + ";" + lon + ";" + orientation;
//    Serial.println(messagetopi);
//  }
//
//  //car person collision
//  else if (accidentforce > 650 && accidentforce <= 999)
//  { state = 1;
//    messagetopi = "impact=2;lat=456.78;long=91011.12;";
//    Serial.println(messagetopi);
//  }
//  if (state == 1)
//  {
//    check = Serial.read();
//    if (check == '$')
//    {
//      Serial.println("lock");
//      stall();
//    }
//  }

  //force sensor pressure for the drivers presence



 

  //calliberation of the accelerometer
  updatevalues();



  //accelerometer reading
  if (state == 0)
  {
    if (vx > (xsum * 93) / 100 && vx < (xsum * 107) / 100 && vy > (ysum * 93) / 100 && vy < (ysum * 107) / 100 && vz > (zsum * 93) / 100 && vz < (zsum * 107) / 100 )
    {
      //Serial.println("Straight");
      orientation = 1;
    }
    else if (vy > (ysum * (100 + incper + 10)) / 100 && vz < (zsum * (100 - decper - 10)) / 100)
    {
      //messagetopi = "impact=3;lat=456.78;long=91011.12;";
      orientation = 2;
      //Serial.println(messagetopi);
    }
    else if (vy < (ysum * (100 - decper)) / 100 && vz < (zsum * (100 - decper)) / 100)
    {
//      messagetopi = "impact=3;lat=456.78;long=91011.12;";
//      Serial.println(messagetopi);
      orientation = 3;
//      Serial.println("Backward flip");
    }
    else if (vx > (xsum * (100 + incper)) / 100 && vz < (zsum * (100 - decper)) / 100)
    {
//      messagetopi = "impact=3;lat=456.78;long=91011.12;";
//      Serial.println(messagetopi);
      orientation = 4;
//      Serial.println("right Flip");
    }
    else if (vx < (xsum * (100 - decper)) / 100 && vz < (zsum * (100 - decper)) / 100)
    { messagetopi = "impact=3;lat=456.78;long=91011.12;";
//      Serial.println(messagetopi);
//      Serial.println("Left Flip");
      orientation = 5;
    }
    else
    {
     /// messagetopi = "impact=3;lat=456.78;long=91011.12;";
//      Serial.println(messagetopi);
//      Serial.println("Flip Over");
         orientation = 6;
    }

  }
 


  //direction of motion reading
 
  delay(2000);
}

//void gpsdump(TinyGPS &gps)
//{
//
//  long lat, lon;
//  float flat,flon;
//  unsigned long age;
//
//  gps.get_position(&lat, &lon);
//  Serial.print("Lat/Long(10^-5 deg): ");
//  Serial.print(lat);
//  Serial.print(", ");
//  Serial.print(lon);
//
//  gps.f_get_position(&flat, &flon, &age);
//  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
//}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
