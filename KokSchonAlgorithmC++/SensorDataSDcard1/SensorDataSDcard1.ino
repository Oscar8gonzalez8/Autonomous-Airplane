#include <Quaternion2Euler.h>
#include <function_CppKokSchonQuaternionEstimator.h>
#include <SPI.h>
#include <SD.h>
#include <MPU9250_WE.h>
#include <BMP280_DEV.h>
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
bool validGPSDataReceived = false;

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

#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
BMP280_DEV bmp280;

float temperature, pressure, altitude;

bool USBconnected = true;
double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
double t_last = 0.0;

void setup() {
  Serial.begin(9600);
  delay(3000);
  if (!Serial) {
    USBconnected = false;
  }
  if (useGPS) {
    ss.begin(9600);
    unsigned long GPSdelay = 1000;
    readGPS(GPSdelay);
  }
  Wire.begin();
  delay(2000);
  if (!myMPU9250.init()) {
    if (USBconnected) {
      Serial.println("MPU9250 does not respond");
    }
  }
  if (!myMPU9250.initMagnetometer()) {
    if (USBconnected) {
      Serial.println("Magnetometer does not respond");
    }
  }
  bmp280.begin();
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);
  bmp280.startNormalConversion();

  if (USBconnected) {
    Serial.println("Position your MPU9250 flat and don't move it - calibrating..");
  }
  delay(1000);
  myMPU9250.autoOffsets();
  if (USBconnected) {
    Serial.print("Done!");
  }
}

void loop() {
  xyzFloat accel = myMPU9250.getGValues();
  xyzFloat gyro = myMPU9250.getGyrValues();
  xyzFloat Mag = myMPU9250.getMagValues();

  if (useGPS) {
    unsigned long GPSdelay = 1;
    readGPS(GPSdelay);
    unsigned long age;
    gps.f_get_position(&GPSlat, &GPSlon, &age);
  }

  if (GPSlat != 1000.0 && GPSlon != 1000.0) {
    if (!validGPSDataReceived) {
      initialTimestamp = millis();
      validGPSDataReceived = true;
    }
    
    double t = (double)(millis() - initialTimestamp) / 1000.0;
    double dt = t - t_last;
    t_last = t;

    // Assuming that quaternion processing and conversion to Euler angles is done here
    // The conversion of the quaternion to yaw, pitch, and roll might be different for you.
    double yaw, pitch, roll;
    // ... Your quaternion processing ...

    String dataString = "";
    dataString += t;
    dataString += ",";
    dataString += accel.x;
    dataString += ",";
    dataString += accel.y;
    dataString += ",";
    dataString += accel.z;
    dataString += ",";
    dataString += gyro.x;
    dataString += ",";
    dataString += gyro.y;
    dataString += ",";
    dataString += gyro.z;
    dataString += ",";
    dataString += pressure;
    dataString += ",";
    dataString += Mag.x;
    dataString += ",";
    dataString += Mag.y;
    dataString += ",";
    dataString += Mag.z;
    dataString += ",";
    dataString += yaw;
    dataString += ",";
    dataString += pitch;
    dataString += ",";
    dataString += roll;
    dataString += ",";
    if (useGPS) {
      char charArray[12];
      dtostrf(GPSlat, 1, 6, charArray);
      dataString += charArray;
      dataString += ",";
      dtostrf(GPSlon, 1, 6, charArray);
      dataString += charArray;
      dataString += ",";
      dataString += GPSspeed;
    } else {
      dataString += "0,0,0";
    }

    writeToSDcard(dataString);
  }
}

void writeToSDcard(String dataString) {
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    if (USBconnected) {
      Serial.println("Failed to open the file");
    }
  }
}

static void readGPS(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available()) {
      char c = ss.read();
      if (gps.encode(c)) {
        // Successful reading
      }
    }
  } while (millis() - start < ms);
}
