#include "MPU9250.h"
#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library
#include <SPI.h>
#include <SD.h>
 
float temperature, pressure, altitude;            // Create the temperature, pressure and altitude variables
BMP280_DEV bmp280;                                // Instantiate (create) a BMP280_DEV object and set-up for I2C operation (address 0x77)
MPU9250 mpu;                                      // Create an MPU9250 Object

const int chipSelect = 17;
File dataFile;

void setup() {
    Serial.begin(115200);                         //Begin serial 
    Wire.begin();                                 //Begin I2C
    delay(2000);

    if (!mpu.setup(0x68)) {                       //Initialize MPU9250
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

  bmp280.begin();                                 // Default initialisation, place the BMP280 into SLEEP_MODE 
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);     // Set the standby time to 2 seconds
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE  

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
    dataFile.println("Gyro0, Gyro1, Gyro2, AccelX, AccelY, AccelZ");  
}

void loop() {
    String dataString = "";
    if (mpu.update()) {                           //Endlessly loop until an update is available 
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            dataString += mpu.getGyroX();
            dataString += ","; 
            dataString += mpu.getGyroY();
            dataString += ","; 
            dataString += mpu.getGyroZ();
            dataString += ","; 
            dataString += mpu.getAccX()*9.806;
            dataString += ","; 
            dataString += mpu.getAccY()*9.806;
            dataString += ","; 
            dataString += mpu.getAccZ()*9.806;
            dataFile.println(dataString);
            dataFile.flush();
            //print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {

  Serial.println();
  Serial.println("/-------------------------------------------------------------/");
  Serial.print("Roll (): "); Serial.print(mpu.getRoll());
  Serial.print("    Pitch : "); Serial.print(mpu.getPitch());
  Serial.print("    Yaw : "); Serial.print(mpu.getYaw());
  Serial.println();
  Serial.print("Acceleration (m/s^2): X : "); Serial.print(mpu.getAccX()*9.806);
  Serial.print("    Acceleration (m/s^2): Y : "); Serial.print(mpu.getAccY()*9.806);
  Serial.print("    Acceleration (m/s^2): Z : "); Serial.print(mpu.getAccZ()*9.806);
  Serial.println();
  Serial.print("Gyroscope (°/sec): X : "); Serial.print(mpu.getGyroX());
  Serial.print("       Gyroscope (°/sec): Y : "); Serial.print(mpu.getGyroY());
  Serial.print("       Gyroscope (°/sec): Z : "); Serial.print(mpu.getGyroZ());
  Serial.println();
  Serial.print("Magnetic: X : "); Serial.print(mpu.getMagX());
  Serial.print("      Magnetic: Y : "); Serial.print(mpu.getMagY());
  Serial.print("      Magnetic: Z : "); Serial.print(mpu.getMagZ());
  Serial.println();
  bmp280.getMeasurements(temperature, pressure, altitude);//Acquire the BMP820 data
  Serial.print("Pressure (hPa) : "); Serial.print(pressure);
  Serial.print("     Altitude (m) : "); Serial.print(altitude);
  Serial.println();  
  Serial.print("Temperature(BMP820) : "); Serial.print(temperature);
  Serial.println();  
  Serial.print("Temperature(MPU9250) : "); Serial.print(mpu.getTemperature()); //The MPU temperature is not acurate or precise, use BMP temp
  Serial.println();  
  delay(100);

}
