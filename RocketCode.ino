#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU9255.h>
#include <TinyGPS++.h>
#define GPSBaud 9600
#define XbeeBaud 9600
#define gpsSS Serial2
#define xbee Serial3

const float alpha = 0.5;
double fXa = 0;
double fYa = 0;
double fZa = 0;
double fXg = 0;
double fYg = 0;
double fZg = 0;
double fXm = 0;
double fYm = 0;
double fZm = 0;


Adafruit_BMP280 bme;
MPU9255 mpu;
TinyGPSPlus gps;



void setup() {

  delay(4000);
  Serial.begin(115200);
  TestSensors();

}

void loop() {
   UpdateTemp();
   delay(100);
   UpdateGyroAccMag();
   delay(100);
     updateGps();
     delay(100);
     Serial.println();
   
    
    delay(1500);
  

}

void TestSensors(){
 if(!mpu.init())
  {
  Serial.println("MPU Not Found");
  }
   mpu.set_acc_bandwidth(acc_5Hz);
   mpu.set_gyro_bandwidth(gyro_5Hz);
   gpsSS.begin(GPSBaud);
   xbee.begin(XbeeBaud);
   if (!bme.begin()) {  
    Serial.println("BMP280 Not Found");
  }


  
  
 }

 void UpdateTemp(){
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    xbee.print(bme.readTemperature());
    Serial.println(" *C");
    xbee.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure());
    Serial.println(" Pa");

    Serial.print("Approx altitude = ");
    Serial.print(bme.readAltitude(1010.25)); // this should be adjusted to your local forcase
    Serial.println(" m");
  
  
  }

  void UpdateGyroAccMag() {
  mpu.read_acc();// read data from accelerometer
  mpu.read_gyro();// get data from gyroscope
  mpu.read_mag();// get data from the magnetometer 

  double pitch, roll,yaw;
    
 
    //Low Pass Filter
    fXa = mpu.ax * alpha + (fXa * (1.0 - alpha));
    fYa = mpu.ay * alpha + (fYa * (1.0 - alpha));
    fZa = mpu.az * alpha + (fZa * (1.0 - alpha));
    fXg = mpu.gx * alpha + (fXg * (1.0 - alpha));
    fYg = mpu.gy * alpha + (fYg * (1.0 - alpha));
    fZg = mpu.gz * alpha + (fZg * (1.0 - alpha));
    fXm = mpu.mx * alpha + (fXm * (1.0 - alpha));
    fYm = mpu.my * alpha + (fYm * (1.0 - alpha));
    fZm = mpu.mz * alpha + (fZm * (1.0 - alpha));
 
    //Roll & Pitch Equations
    roll  = atan2(-fYg, fZg);
    pitch = atan2(fXg, sqrt(fYg*fYg + fZg*fZg));

   yaw = (double)atan2(fZm * sin(roll) - fYm * cos(roll), \
                                      fXm * cos(pitch) + \
                                      fYm * sin(pitch) * sin(roll) + \
                                      fZm * sin(pitch) * cos(roll));
    roll  = (roll*180.0)/M_PI;
    pitch = (pitch*180.0)/M_PI;
    yaw = (yaw*180.0)/M_PI;
 
   

    Serial.print("Ax = ");
    Serial.print(fXa);
    Serial .print(" ");
    Serial.print("Ay = ");
    Serial.print(fYa);
    Serial .print(" ");

    Serial.print("Az = ");
    Serial.print(fZa); 
    Serial .print(" ");
    
    Serial.print("Gx = ");
    Serial.print(fXg);
    Serial .print(" ");
    Serial.print("Gy = ");
    Serial.print(fYg);
    Serial .print(" ");
    Serial.print("Gz = ");
    Serial.print(fZg);
    Serial .print(" ");

    Serial.print("Mx = ");
    Serial.print(fXm);
    Serial .print(" ");
    Serial.print("My = ");
    Serial.print(fYm);
    Serial .print(" ");
    Serial.print("Mz = ");
    Serial.print(fZm);
    Serial .println();
     
   
    Serial.print("pitch = ");
    Serial.println(pitch);
   
    Serial.print("roll = ");
    Serial.println(roll);
    
    Serial.print("yaw = ");
    Serial.println(yaw);

    xbee.print("pitch = ");
    xbee.println(pitch);
   
    xbee.print("roll = ");
    xbee.println(roll);
    
    xbee.print("yaw = ");
    xbee.println(yaw);


  
}

void updateGps() {
 // Serial.println("update gps");
 // xbee.println("Gps");
  while (gpsSS.available() > 0)
  {
     Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----"));
    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
  smartDelay(1000);
    
  }
    

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSS.available())
      gps.encode(gpsSS.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
    xbee.print('*');
    xbee.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    xbee.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
      xbee.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  xbee.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
    xbee.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
    xbee.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
    xbee.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
    xbee.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i){
    Serial.print(i<slen ? str[i] : ' ');
    xbee.print(i<slen ? str[i] : ' ');
    }
  smartDelay(0);
}
