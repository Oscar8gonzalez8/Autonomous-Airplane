#include <Quaternion2Euler.h>
#include <function_CppKokSchonQuaternionEstimator.h>

#include <SPI.h>

#include <SD.h>

#include <MPU9250_WE.h>

#include <BMP280_DEV.h>

//#include <function_CppKokSchonQuaternionEstimator.h>

//#include <Quaternion2Euler.h>

#include <SoftwareSerial.h>

#include <TinyGPS.h>

const String filename = "RollPitchYawData.csv";

bool useGPS = true; 

TinyGPS gps; 
const int rxPin = 1; 
const int txPin = 0; 

SoftwareSerial ss(rxPin, txPin);
static void readGPS(unsigned long ms);
float GPSlat = 0.0; 
float GPSlon = 0.0; 
float GPSspeed = 0.0; 
unsigned long initialTimestamp = 0;

#if !defined(ARDUINO_ARCH_RP2040)
#error For RP2040 only 
#endif 
#if defined(ARDUINO_ARCH_MBED)
#define PIN_SD_MOSI PIN_SPI_MOSI
#define PIN_SD_MISO PIN_SPI_MISO
#define PIN_SD_SCK PIN_SPI_SCK
#define PIN_SD_SS PIN_SPI_SS
#else 
#define PIN_SD_MOSI PIN_SPI0_MOSI
#define PIN_SD_MISO PIN_SPI0_MISO
#define PIN_SD_SCK PIN_SPI0_SCK
#define PIN_SD_SS PIN_SPI0_SS
#endif 

#define _RP2040_SD_LOGLEVEL_ 0 
// ## End of Variables for the SD Card Reader


// ## Start of variables for the 10 DOF MPU9250 Sensor 
#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
BMP280_DEV bmp280; 

float temperature, pressure, altitude; 
// ## End of variables for the 10 DOF MPU9250 

// Additional variables 
bool USBconnected = true; 
double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
double t_last = 0.0; 

void setup(){
  Serial.begin(9600);
  delay(3000);
  if (!Serial){
    USBconnected = false; 
  }
  if (useGPS){
    ss.begin(9600);
    unsigned long GPSdelay = 1000; // Time delay 
    readGPS(GPSdelay);
  }
  Wire.begin(); // Being I2C
  delay(2000);
  if (!myMPU9250.init()){
    if (USBconnected){
      Serial.println("MPU9250 does not respond");
    }
  }
  if (!myMPU9250.initMagnetometer()){
    if (USBconnected){
      Serial.println("Magnetometer does not respond");
    }
  }
  bmp280.begin(); // Default initialization
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);
  bmp280.startNormalConversion();

  if (USBconnected){
    Serial.println("Position your MPU9250 flat and don't move it - calibrating..");
  }
  delay(1000);
  myMPU9250.autoOffsets(); // Callibrate the accelerometer and gyro offsets
  if (USBconnected){
    Serial.print("Done!");
  }
#if defined(ARDUINO_ARCH_MBED)
  if (USBconnected){
    Serial.print("Starting SD Card ReadWrite on MBED");
  }
#else 
if (USBconnected){
  Serial.print("Starting SD Card ReadWrite on");
}
#endif 
if (USBconnected){
  Serial.println(BOARD_NAME);
  Serial.print("Initializing SD card with SS = ");
  Serial.println(PIN_SD_SS);
  Serial.print("SCK = ");
  Serial.println(PIN_SD_SCK);
  Serial.print("MOSI = ");
  Serial.println(PIN_SD_MOSI);
  Serial.print("MISO = ");
  Serial.println(PIN_SD_MISO);
}
if (!SD.begin(PIN_SD_SS)){
  if (USBconnected){
    Serial.println("Initialization failed!");
  }
}
if (USBconnected){
  Serial.println("Initialization done.");
}

File dataFile = SD.open(filename, FILE_WRITE);
if (dataFile){
    String headerString = "time,";
    headerString += "gFx,gFy,gFz,wx,wy,wz,p,Bx,By,Bz,Azimuth,Pitch,Roll,Latitude,Longitude,Speed (m/s)";
    dataFile.println(headerString);
    dataFile.close();
}
else {
  if (USBconnected){
    Serial.println("Initialization failed to open the file");
  }
}
t_last = 0.0;

}

void loop(){
  xyzFloat accel = myMPU9250.getGValues();
  //Get the values from the Gyrometer
  xyzFloat gyro = myMPU9250.getGyrValues();
  //Get the values from the Magnetometer
  xyzFloat Mag = myMPU9250.getMagValues();

  double gyrometer[3];
  double gravity[3];
  double magnetometer[3];
  gyrometer[0] = gyro.x * 3.14159 / 180.0; 
  gyrometer[1] = -gyro.y * 3.14159 / 180.0; 
  gyrometer[2] = -gyro.z * 3.14159 / 180.0; 

  gravity[0] = -accel.x; 
  gravity[1] = accel.y; 
  gravity[2] = accel.z; 

  magnetometer[0] = (Mag.y - 40.3058);
  magnetometer[1] = -(Mag.x - 40.3058);
  magnetometer[2] = (Mag.z - 40.3058);

  if (initialTimestamp == 0) {
        initialTimestamp = millis();
    }

  // Get the relative time in seconds
  double t = (double)(millis() - initialTimestamp) / 1000.0;
  double dt = t - t_last;
  t_last = t;

  function_KokSchonQuaternionEstimator(
    quaternion, 
    quaternion, 
    gyrometer, 
    gravity, 
    magnetometer, 
    dt, 
    67.0, 
    1.0, 
    0.3
  );
  double roll, pitch, yaw; 
  Quaternion2Euler(&roll, &pitch, &yaw, quaternion);
  bmp280.getMeasurements(temperature, pressure, altitude);

  if (useGPS){
    unsigned long GPSdelay = 1; 
    readGPS(GPSdelay);
    unsigned long age; 
    gps.f_get_position(&GPSlat, &GPSlon, &age);
  }

String dataString = ""; 
dataString += t; 
dataString += ",";
dataString += gravity[0];
dataString += ",";
dataString += gravity[1];
dataString += ",";
dataString += gravity[2];
dataString += ",";
dataString += gyrometer[0];
dataString += ",";
dataString += gyrometer[1];
dataString += ",";
dataString += gyrometer[2];
dataString += ",";
dataString += pressure;
dataString += ",";
dataString += magnetometer[0];
dataString += ",";
dataString += magnetometer[1];
dataString += ",";
dataString += magnetometer[2];
dataString += ",";
dataString += yaw;
dataString += ",";
dataString += pitch;
dataString += ",";
dataString += roll;
dataString += ",";
if (useGPS){
  char charArray[12];
  dtostrf(GPSlat, 1, 6, charArray);
  dataString += charArray; 
  dataString += ",";
  dtostrf(GPSlon, 1, 6, charArray);
  dataString += charArray; 
  dataString += ",";
  dataString += GPSspeed; 
}
else{
  dataString += "0";
  dataString += ",";
  dataString += "0";
  dataString += ",";
  dataString += "0";
}

writeToSDcard(dataString);
}

void writeToSDcard(String dataString){
  File dataFile = SD.open(filename, FILE_WRITE);
  Serial.println(dataString);
  if (dataFile){
    dataFile.println(dataString);
    dataFile.close();
  }
  else{
    if (USBconnected){
      Serial.println("Failed to open the file");
    }
  }
}

static void readGPS(unsigned long ms){
  unsigned long start = millis();
  do {
    while (ss.available())
    gps.encode(ss.read());
  }
  while(millis() - start < ms);
}