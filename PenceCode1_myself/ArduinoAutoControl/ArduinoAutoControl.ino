#include <AutoControl_PENCE.h>
#include <Quaternion2Euler_PENCE.h>
#include <function_CppKokSchonQuaternionEstimator_PENCE.h>

#include <SPI.h>
#include <SD.h>
#include <MPU9250_WE.h>
#include <BMP280_DEV.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Servo.h>

// Name the file for the SD card
String filename = "FlightData001.csv";

// Say whether to collect GPS data
bool useGPS = true;

// ################Varibales for GPS Flight ###############################
TinyGPS gps;
const int rxPin = 1;
const int txPin = 0;
SoftwareSerial ss(rxPin, txPin);
static void readGPS(unsigned long ms);
float GPSlat;
float GPSlon;
float GPSspeed;
double target_lat = 43.382093192;
double target_lon = -111.7852215;
double target_alt = 854.0;

double target_lat1 = 83.820934;
double target_lon1 = -111.785268;
double target_alt1 = 854.0;

double target_lat2 = 83.820934;
double target_lon2 = -111.785268;
double target_alt2 = 854.0;

double target_lat3 = 83.820934;
double target_lon3 = -111.785268;
double target_alt3 = 854.0;

double target_lat4 = 83.820934;
double target_lon4 = -111.785268;
double target_alt4 = 854.0;

bool GPSisValid = true;

// ####################end of GPS#################################

// ################Varible for SD Card Reader#####################
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

// ############ End of Variables for the SD Card Reader#############

// ############## Start of variables for the 10 DOF MPU9250 Sensor#####
#define MPU9250_ADDR 0x68
// Create the MPU9250 object and name it myMPU9250
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
BMP280_DEV bmp280;

float temperature, pressure, altitude;
// ############ End of variables for the 10 DOF MPU9250 ######################

// ############Varibles for the Servos #####################################
Servo AileronServo;  // create servo object to control a servo
Servo RudderServo;
Servo ElevatorServo; 
const int AileronPin = 10;
const int ElevatorPin = 12;
const int RudderPin = 11;
const double kp_roll = 10.0;
const double kd_roll = 0.0;
const double kp_pitch = -10.0;
const double kp_yaw = 5.;
const double kp_rudder = 0.2;
const double max_roll = 3.14 / 4.0;
const double max_pitch = 3.14 / 4.0;

// Additional variables
bool USBconnected = true;
double quaternion[4] = { 1.0, 0.0, 0.0, 0.0 };
double t_last = 0.0;
unsigned long initialTimestamp = 0;

void setup() {
  // Start serial communication
  Serial.begin(9600);
  delay(3000);

  if (!Serial) {
    USBconnected = false;
  }
  if (useGPS) {
    ss.begin(9600);
    unsigned long GPSdelay = 1000;  // Time delay
    readGPS(GPSdelay);
  }
  Wire.begin();  // Begin I2C
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
  bmp280.begin();  // Default initialization, place the BMP280 into SLEEP_MODE
  // bmp280.setPresOversampling(OVERSAMPLING_X4); //Set the oversampling to X4
  // bmp280.setPresOversampling(OVERSAMPLING_X1); //Set the oversampling to X1
  // bmp280.setIRRFilter(IIR_FILTER_4);//Set the IIR filter setting to 4
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);
  bmp280.startNormalConversion();

  if (USBconnected) {
    Serial.println("Position your MPU9250 flat and don't move it - calibrating..");
  }
  delay(1000);
  myMPU9250.autoOffsets();  // Calibrate the accelerometer and gyro offsets
  if (USBconnected) {
    Serial.println("Done!");
  }
#if defined(ARDUINO_ARCH_MBED)
  if (USBconnected) {
    Serial.println("Starting SD Card ReadWrite on MBED");
  }
#else
  if (USBconnected) {
    Serial.println("Starting SD Card ReadWrite on");
  }
#endif
  if (USBconnected) {
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
  if (!SD.begin(PIN_SD_SS)) {
    if (USBconnected) {
      Serial.println("Initialization failed!");
    }
  } else {

    for (int ii = 0; ii < 10; ii++) {
      if (!SD.exists(filename)) {
        filename = "Flight";
        filename += String(ii);
        filename += ".csv";
      }
    }
  }
  if (USBconnected) {
    Serial.println("Initialization done.");
  }
  // Create the SD card file and open it for writing
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    String headerString = "time,";
    headerString += "gFx,gFy,gFz,wx,wy,wz,p,Bx,By,Bz,Azimuth,Pitch,Roll, Latitude,Longitude,Speed (m/s),delta A, delta E, delta r";
    dataFile.println(headerString);
    dataFile.close();
  } else {
    if (USBconnected) {
      Serial.println("Initialization failed to open the file");
    }
  }

  AileronServo.attach(AileronPin);
  ElevatorServo.attach(ElevatorPin);
  RudderServo.attach(RudderPin);
  t_last = 0.0;
}

void loop() {
  double alt;

  if (initialTimestamp == 0) {
    initialTimestamp = millis();
  }
  double time = (double)(millis() - initialTimestamp) / 1000.0;
  double dt = time - t_last;
  t_last = time;

  xyzFloat accel = myMPU9250.getGValues();
  // Get the values from the Gyrometer
  xyzFloat gyro = myMPU9250.getGyrValues();
  // Get the values from the Magnetometer
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


  double beta_val[3] = {-44.9412,6.6114,-61.8925};

  magnetometer[0] = (Mag.y - beta_val[0]);
  magnetometer[1] = -(Mag.x - beta_val[1]);
  magnetometer[2] = (Mag.z - beta_val[2]);

  function_KokSchonQuaternionEstimator(
    quaternion,
    quaternion,
    gyrometer,
    gravity,
    magnetometer,
    dt,
    67.0,
    1.0,
    0.3);
  // *delta_e = kp_pitch * pitch_err;
  double roll, pitch, yaw;
  Quaternion2Euler(&roll, &pitch, &yaw, quaternion);
  // Get the bmp280 measurements
  bmp280.getMeasurements(temperature, pressure, altitude);

  if (useGPS) {
    unsigned long GPSdelay = 1;
    readGPS(GPSdelay);
    unsigned long age;
    gps.f_get_position(&GPSlat, &GPSlon, &age);
    if (((GPSlat - 43) < 2) && ((GPSlon + 111.0) < 2) && useGPS) {
      GPSisValid = true;
    } else {
      GPSisValid = false;
    }
  }
  double delta_t;
  double delta_a;
  double delta_e;
  double delta_r;

  double target_lat = target_lat;
  double target_lon = target_lon;
  double target_alt = target_alt;
  if (GPSisValid) {
    AutoControl(
      &delta_t,
      &delta_e,
      &delta_a,
      &delta_r,
      quaternion,
      gyrometer,
      target_lat,
      target_lon,
      target_alt + 2.0,
      GPSlat,
      GPSlon,
      target_alt,
      kp_roll,
      kd_roll,
      kp_pitch,
      kp_yaw,
      kp_rudder,
      max_roll,
      max_pitch);
  }


  else {
    delta_e = -kp_pitch * pitch;
    delta_a = -kp_roll * roll;
    delta_r = -kp_rudder * yaw;
  }
  delta_e = -kp_pitch * pitch;
  const double delta_e_trim = -0.13;
  ElevatorServo.write(mapServoCmd((delta_e - delta_e_trim)));
  AileronServo.write(mapServoCmd(delta_a));
  RudderServo.write(mapServoCmd(delta_r));

  String dataString = "";
  dataString += time;
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
  dataString += delta_a;
  dataString += ",";
  dataString += delta_e;
  dataString += ",";
  dataString += delta_r;
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
    dataString += "0";
    dataString += ",";
    dataString += "0";
    dataString += ",";
    dataString += "0";
  }

  writeToSDcard(dataString);
}

void writeToSDcard(String dataString) {
  File dataFile = SD.open(filename, FILE_WRITE);
  Serial.println(dataString);
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
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static int mapServoCmd(double delta) {
  double slope = (135.0 - 90.0) * delta;
  return (((int)slope) + 90.0);
}
