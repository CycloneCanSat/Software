// This is designed for the Advanced Category of the United Kingdom CanSat
// Competition 2016, as part of Cyclone, a team from St Paul's School,
// Barnes, London.
//
// For more information about the entry, including the Electronics Design
// Take a look on the GitHub Page - http://github.com/cyclonecansat or our
// website at: http://teamcycl.one
//
// Any questions, contact me at: ashwin.ahuja@gmail.com

/*
 * INCLUDES
 * 
 * Sensor Library - Library for the reading and manipulation of the MS5637
 * and HYT271 - the pressure, internal temperature (MS5637) and humidity 
 * and external temperature (HYT271) sensors
 * 
 * RFM98W Library - Library for the RFM98W Transceiver Board, using the LoRa
 * transmission protocol - including dealing with creation of the Packet and 
 * the transmission and receiving of data.
 * 
 * Servo Library - Library used for the release of the parachute, currently 
 * unused, since it will likely not be used.
 * 
 * Wire Library - Library required for all I2C communication.
 * 
 * TinyGPS++ Library - Library used for receiving and interpreting data from 
 * the GP-2106 module (available from Sparkfun) and used alongside the board
 * produced by Sparkfun - the GP-2106 Evaluation Board
 * 
 * SPI - Library used for all SPI communications - currently unused, but could
 * be used for the 9DOF sensor.
 */
#include <sensor_library.h> 
#include <RFM98W_library.h>
#include <Servo.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
/*
 * DEFINITIONS
 * 
 * Pins for the RFM98W - as implied
 * Hardware Serial declarations for the GPS module and the OpenLog
 * Number of bytes implied by each command
 * Parachute Release Servo Pin
 * Boolean - parachute released
 * Boolean - parachute armed
 * Parachute Servo - move from (minimum)
 * Parachute Servo - move to (maximum)
 * Baud Rate for GPS communication
 * Baud Rate for Computer Serial communications
 * Verbosity - 0 for none, 1 for all
 * Boolean - Longitude
 * Boolean - Latitude
 * Time between transmissions (during other times, the system will be receiving data from the RFM98W
 * Various pieces of information about the program
 */
//LoRa Pins
#define nss 20
#define dio0 7
#define dio5 16
#define rfm_rst 21

// Serial Declarations
#define gpsSerial Serial1
#define openlog Serial2

//radio command lengths
const byte cmd_lengths[8] = {0,8,2,1,1,2,2,2};

// Servo Info
const int ServoPin = 0; // NOT CORRECT
bool parachuteReleased = false;
bool parachuteArmed = false;
const int servoMin = 0;
const int servoMax = 180;

// Baud Rate Declarations
#define GPS_BaudRate 4800
#define Computer_BaudRate 115200
#define OpenLogBaudRate 9600

//Verbosity
#define verbosity 1

//LSM9DS1 definitions
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED

//GPS Declarations
bool latitudePositive = false;
bool longitudePositive = true;

//Time between transmissions
#define timeBetweenTransmissions 1000

//Various other info
#define SoftwareVersionNumber "1.0.0"
#define Author "Ashwin Ahuja"

/*
 * OBJECT DECLARATIONS
 * 
 * TinyGPSPlus Object - receives and decodes GPS data
 * SensLib - Communication with MS5637 and HYT271
 * RFMLib - Communication with Hope RFM98W
 * LSM9DS1 -  Communication with 9DOF
 * Servo - Parachite release
 * Radio transmission timer.
 * Sensor reading timer
 * Packet received boolean
 * Has MS5637 been read?
 * Is it time to read?
 */

TinyGPSPlus gps; // GPS
SensLib sns; // Sensor Object
RFMLib radio = RFMLib(nss, dio0, dio5, rfm_rst); // Radio Object
LSM9DS1 imu; // LSM9DS1 Object
Servo ParaRelease; // Parachute Release Servo Object
uint32_t radioTransmitTimer; 
uint32_t sensorReadTimer;
boolean pkt_rx = false;
boolean msRead = false;
boolean readSens = false;

/*
 * Miscellaneous Declarations
 */
int gyx;
int gyy;
int gyz;
int acx;
int acy;
int acz;
int magx;
int magy;
int magz;
float roll;
float pitch;
float heading;

void assemblePacket (RFMLib:: Packet &pkt);
void decodePacket (RFMLib:: Packet pkt);

void setup() {
  ParaRelease.attach(ServoPin);
  ParaRelease.write(servoMin);
  Serial.begin(Computer_BaudRate);
  gpsSerial.begin(GPS_BaudRate);
  openlog.begin(OpenLogBaudRate);
  SPI.begin();
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);//Radio configuration
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (imu.begin())
  {
    #if verbosity == 1
    Serial.println("9DOF Initialised");
    #endif
  }
  radioTransmitTimer = millis();
  #if verbosity != 0
  Serial.print("Cyclone CanSat Firmware Version Number: ");
  Serial.print(SoftwareVersionNumber);
  Serial.print(" (Atlantis). Verbosity is equal to ");
  Serial.println(verbosity);
  #endif
}

void loop() {
  readIMU();
}

void readIMU()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();
  gyx = imu.calcGyro(imu.gx);
  gyy = imu.calcGyro(imu.gy);
  gyz = imu.calcGyro(imu.gz);
  acx = imu.calcAccel(imu.ax);
  acy = imu.calcAccel(imu.ay);
  acz = imu.calcAccel(imu.az);
  magx = imu.calcMag(imu.mx);
  magy = imu.calcMag(imu.my);
  magz = imu.calcMag(imu.mz);
  calculateHeadingAndRoll();
}

void calculateHeadingAndRoll()
{
  #define DECLINATION -8.58
  roll = atan2(imu.ay, imu.az);
  pitch = atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az));
  
  heading;
  if (imu.my == 0)
    heading = (imu.mx < 0) ? 180.0 : 0;
  else
    heading = atan2(imu.mx, imu.my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
}

