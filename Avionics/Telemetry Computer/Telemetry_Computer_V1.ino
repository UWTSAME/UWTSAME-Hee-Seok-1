// altimeter set up
#include <Adafruit_MPL3115A2.h>
#include <Wire.h>
Adafruit_MPL3115A2 myAlt;
#define ALTBASIS

float altitude = 0;
float elapsed_time =0;

// Accelerometer set up
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100) //the IMU will wait 100 ms between re-samples
Adafruit_BNO055 myIMU = Adafruit_BNO055(); // sets myIMU up

float dt; //change in time 
unsigned long milli_second_old;
float velocity = 0, filteredVelocity = 0;

// SD card set up
#include <SD.h>
#include <SPI.h>
int chip_select = 10; // pin used to talk to SD card
bool THISISDUMB = false;
File SD_card;

// save to SD card every 0.1 secods, create a list of data between saves.
int folderNum = 0;
String folderPath = "";

float time_count = 0;

void folderMaker () {
  while (true) {
    folderPath = "/" + String(folderNum);

    if (!SD.exists(folderPath)){
      SD.mkdir(folderPath);
      break;
    }
    folderNum++;
  }
}


void setup() {
  // put your setup code here, to run once:
 /// use gravity vector 
 pinMode(LED_BUILTIN, OUTPUT);
  // turn the LED on (HIGH is the voltage level)
  Serial.begin(115200);
  Wire.begin(); // maakes ready I2C communication 
  myIMU.begin(); // boots up IMU
  delay(1000); // this pause allows time for the IMU to boot up
  myAlt.begin(); // boots up Altimeter
  delay(500);
  
  myAlt.setSeaPressure(1015.3); // set this to current sea level pressure in hPa, used to calibrate alt.

  myIMU.setExtCrystalUse(true); // uses different crystal clock thats more accurate on the IMU
   
  pinMode(chip_select, OUTPUT); // chip select pinmode for SD card
  
  //////////////////  Start of IMU calibration /////////////////////////////////
  uint8_t system, gyro, accel, mag = 0; // sets up calibration level


  do{  // runs the calibration untill each value reaches full calibration which is = to 3
    myIMU.getCalibration(&system, &gyro, &accel, &mag); //gets calibration levels

    Serial.print("Gyro: ");
    Serial.print(gyro);
    Serial.print("  Mag: ");
    Serial.print(mag);
    Serial.print("  accel: ");
    Serial.print(accel);
    Serial.print("  system: ");
    Serial.println(system);

  }while (((gyro < 3) || (mag < 3) || (accel < 3) || (system < 3)));

// below will flash the built in LED light letting the user know the IMU is calibrated.
for (int i = 0; i < 10; i++) {
    Serial.println("ITS IN");
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(250);
  }

  //////////////////////////// End of IMU calibration //////////////////////////////////////////////////////////////////////////

  

  SD.begin(chip_select);
  folderMaker ();
  SD_card = SD.open("/" + folderPath + "/Data.txt", FILE_WRITE);
  

  milli_second_old = millis();
  Serial.print("GO");
   }


void loop() {
  // put your main code here, to run repeatedly:
  
  //////////////////////////////// Gets IMU data //////////////////////////////////////////////////////////////////////////
  imu::Quaternion quat = myIMU.getQuat();
  int8_t temp = myIMU.getTemp();
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // sets up IMU to return acceleration vector with 3 DOF
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // sets up IMU to return GYRO vector with 3 DOF
  imu::Vector<3> gra = myIMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY); // sets up IMU to return magnetometer vector with 3 DOF
  imu::Vector<3> linAccel = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  ////////////////////////////// Gets altimeter data ////////////////////////////////////////////////////////
    ////////Barometric Pressure Manual Calculation ////////////
  //float pressure = myAlt.getPressure();
  //float seaLevel = 101325.0; //In Pascals
  //float altitude = 44330.0 * (1.0 - pow(pressure / seaLevel, 0.19030));
    ////////////////Altitude Using Library ///////////////////
  //float altitude = myAlt.getAltitude(); // alt in meters

  if (elapsed_time >= 4){
  altitude = myAlt.getAltitude(); // alt in meters

  }
  
  
  dt = (millis() - milli_second_old)/1000.;
  milli_second_old = millis();
  elapsed_time += dt;

  velocity = velocity + linAccel.x() * dt; // calculates velocity from acceleration

  Serial.print(elapsed_time);
  Serial.print(",");
  Serial.println(dt);

  
  SD_card.println(String(dt) + "," + String(velocity) + "," + String(linAccel.x()/9.81) + "," + String(linAccel.y()/9.81) + "," + String(linAccel.z()/9.81) + ","+ String(quat.w()) + "," + String(quat.x()) + "," + String(quat.y()) + "," + String(quat.z()) + "," + String(altitude));
  
  time_count = time_count + 1;
  //Serial.println(dt);
  if (time_count >= 25){
    SD_card.close();
    SD_card = SD.open("/" + folderPath + "/Data.txt", FILE_WRITE);
  }
  
 



  // we need to log all 9 outputs, log 

}



