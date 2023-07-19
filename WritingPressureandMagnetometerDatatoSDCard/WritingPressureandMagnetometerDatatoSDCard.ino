//Include MPU 9250 version 1.2.6. To get it, go to
//Sketch>>Include Library>>Manage Libraries... Search for MPU6250_WE

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

#define RP2040_SD_LOGLEVEL 0

#include <MPU9250_WE.h>
#include <BMP280_DEV.h> // Include the BMP280_DEV.h library
#include <SPI.h>
#include <SD.h>
//Define MPU9250_ADDR to be the I2C address of the MPU 9250
#define MPU9250_ADDR 0x68
float temperature, pressure, altitude; //declare temperature, pressure, and altitude
//Create the MPU9250 object and name it myMPU9250
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
//Create the BMP280_DEV object and name it bmp280. The I2C address is 0x77
BMP280_DEV bmp280;

void writeToSDCard(String dataString);
unsigned long initialTimestamp = 0;
void setup()
{
Serial.begin(9600); //Begin serial
Wire.begin(); //Begin I2C
delay(2000); //Wait 2 seconds

//while (!Serial);
delay(1000);

#if defined(ARDUINO_ARCH_MBED)
  Serial.print("Starting SD Card ReadWrite on MBED ");
#else
  Serial.print("Starting SD Card ReadWrite on ");
#endif

Serial.println(BOARD_NAME);

Serial.print("Initializing SD card with SS = ");
Serial.println(PIN_SD_SS);
Serial.print("SCK = ");
Serial.println(PIN_SD_SCK);
Serial.print("MOSI = ");
Serial.println(PIN_SD_MOSI);
Serial.print("MISO = ");
Serial.println(PIN_SD_MISO);

if (!SD.begin(PIN_SD_SS)) {
    Serial.println("Initialization failed!");
    return;
  }
Serial.println("Initialization done.");

  //create the SD card file and open it for writing
File dataFile = SD.open("SDdata2.csv", FILE_WRITE);

if (dataFile) {
    String headerString = "time,";
    headerString += "gFx,gFy,gFz,wx,wy,wz,p,Bx,By,Bz,Azimuth,Pitch,Roll,Latitude,Longitude,Speed (m/s)";
    dataFile.println(headerString);
  }

if(!myMPU9250.init()){ //Start the MPU, if it fails, report an error
Serial.println("MPU9250 does not respond");
}
else{ //If it succeeds, report success
Serial.println("MPU9250 is connected");
}
if(!myMPU9250.initMagnetometer()){ //Start the magnetometer, if failure, report
Serial.println("Magnetometer does not respond");
}
else{ //if success, report
Serial.println("Magnetometer is connected");
}
/* The slope of the curve of acceleration vs measured values fits quite well to the
* theoretical values, e.g. 16384 units/g in the +/- 2g range. But the starting point,
* if you position the MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The
* autoOffset function measures offset values. It assumes your MPU9250 is positioned
* flat in the x,y-plane. The more you deviate from this, the less accurate will be
* your results.
* The function also measures the offset of the gyroscope data. The gyroscope offset
* does not depend on the positioning.
* This function needs to be called at the beginning since it can overwrite your settings!
*/
Serial.println("Position your MPU9250 flat and don't move it - calibrating...");
delay(1000);
myMPU9250.autoOffsets(); //Callibrate the accelerometer and gyro offsets
Serial.println("Done!");
/* This is a more accurate method for calibration. You have to determine the minimum
* and maximum raw acceleration values of the axes determined in the range +/- 2 g.
* You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
* Use either autoOffset or setAccOffsets, not both.
*/
//myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
/* The gyroscope data is not zero, even if you don't move the MPU9250.
* To start at zero, you can apply offset values. These are the gyroscope raw values you
* obtain using the +/- 250 degrees/s range.
* Use either autoOffset or setGyrOffsets, not both.
*/
//myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);
/* You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF,
* you need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a
* shorter delay, but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
* MPU9250_BW_WO_DLPF_3600
* MPU9250_BW_WO_DLPF_8800
*/
myMPU9250.enableGyrDLPF();
//myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
/* Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
* MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
*
* DLPF Bandwidth [Hz] Delay [ms] Output Rate [kHz]
* 0 250 0.97 8
* 1 184 2.9 1
* 2 92 3.9 1
* 3 41 5.9 1
* 4 20 9.9 1
* 5 10 17.85 1
* 6 5 33.48 1
* 7 3600 0.17 8
*
* You achieve lowest noise using level 6, but it also results in a phase shift
*/
myMPU9250.setGyrDLPF(MPU9250_DLPF_1);
/* Sample rate divider divides the output rate of the gyroscope and accelerometer.
* Sample rate = Internal sample rate / (1 + divider)
* It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
* Divider is a number 0...255
*/
myMPU9250.setSampleRateDivider(5);
/* MPU9250_GYRO_RANGE_250 250 degrees per second (default)
* MPU9250_GYRO_RANGE_500 500 degrees per second
* MPU9250_GYRO_RANGE_1000 1000 degrees per second
* MPU9250_GYRO_RANGE_2000 2000 degrees per second
*/
myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
/* MPU9250_ACC_RANGE_2G 2 g (default)
* MPU9250_ACC_RANGE_4G 4 g
* MPU9250_ACC_RANGE_8G 8 g
* MPU9250_ACC_RANGE_16G 16 g
*/
myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
/* Enable/disable the digital low pass filter for the accelerometer
* If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
*/
myMPU9250.enableAccDLPF(true);
/* Digital low pass filter (DLPF) for the accelerometer, if enabled
* MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
* DLPF Bandwidth [Hz] Delay [ms] Output rate [kHz]
* 0 460 1.94 1
* 1 184 5.80 1
* 2 92 7.80 1
* 3 41 11.80 1
* 4 20 19.80 1
* 5 10 35.70 1
* 6 5 66.96 1
* 7 460 1.94 1
*/
myMPU9250.setAccDLPF(MPU9250_DLPF_1);
/* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
* By default all axes are enabled. Parameters are:
* MPU9250_ENABLE_XYZ //all axes are enabled (default)
* MPU9250_ENABLE_XY0 // X, Y enabled, Z disabled
* MPU9250_ENABLE_X0Z
* MPU9250_ENABLE_X00
* MPU9250_ENABLE_0YZ
* MPU9250_ENABLE_0Y0
* MPU9250_ENABLE_00Z
* MPU9250_ENABLE_000 // all axes disabled
*/
//myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
//myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);
/*
* AK8963_PWR_DOWN
* AK8963_CONT_MODE_8HZ default
* AK8963_CONT_MODE_100HZ
* AK8963_FUSE_ROM_ACC_MODE
*/
myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
delay(200);
bmp280.begin(); // Default initialisation, place the BMP280 into SLEEP_MODE
//bmp280.setPresOversampling(OVERSAMPLING_X4); // Set the pressure oversampling to X4
//bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
//bmp280.setIIRFilter(IIR_FILTER_4); // Set the IIR filter to setting 4
bmp280.setTimeStandby(TIME_STANDBY_2000MS); // Set the standby time to 2 seconds
bmp280.startNormalConversion(); // Start BMP280 continuous conversion in NORMAL_MODE

//For example, only display X, Y, and Z magnetometer data
Serial.println("X, Y, Z");
}


void loop() {
//Get the values from the accelerometer
xyzFloat accel = myMPU9250.getGValues();
//Get the values from the Gyrometer
xyzFloat gyro = myMPU9250.getGyrValues();
//Get the values from the Magnetometer
xyzFloat Mag = myMPU9250.getMagValues();

if (initialTimestamp == 0) {
        initialTimestamp = millis();
    }

// Get the relative time in seconds
double time = (double)(millis() - initialTimestamp) / 1000.0;

//Get the bmp280 measurements
//Temperature in Celsius, pressure in hectopascals, altitude in meters
bmp280.getMeasurements(temperature, pressure, altitude);
//Print the values to the Serial monitor or plotter
String dataString; 
dataString += time; 
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
dataString += "0";
dataString += ",";
dataString += "0";
dataString += ",";
dataString += "0";
dataString += ",";
dataString += "0";
dataString += ",";
dataString += "0";
dataString += ",";
dataString += "0";
/*
 // Construct the dataString
    String dataString = String(time) + "," + accel.x + "," + accel.y + "," + accel.z + "," + 
                        gyro.x + "," + gyro.y + "," + gyro.z + ",0,0,0,0,0,0,0,0,0,0";
*/

writeToSDCard(dataString);
}



void writeToSDCard(String dataString){
  File dataFile = SD.open("SDdata2.csv", FILE_WRITE);
  if (dataFile){
    dataFile.println(dataString);
    dataFile.close();
  }
  else{
    Serial.println("Failed to open file");
  }
}
