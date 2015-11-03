
//21.2.15 -- Solent Sandfish
//Full implementation - CanSat firmware
//To be run on mission hardware as specified in interim reports (PCB Rev.2 )

/* Includes:
 * SPI--LoRa modules.
 * I2C--sensors.
 * TinyGPS++ - GPS data processing
 * SensLib--custom library to read sensor data
 * RFMLib--custom library to perform interface with radio modules
 */

#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <sensor_library.h>
#include <RFM98W_library.h>
// Include the GPS library, then add an instahnce of the GPS sensor, the GPS should be connected to hardware serial,
// By recollection, this is Serial1, but ought to be checked - in fact there are two hardware serials which are broken out - thus either one of them could technically be used
// Alongside the 3.3V and 0V pins which are freely available
// Initialise the sensor at the specific baud rate of the sensor - I think this is likely to be 15200 - but ought to be checked - as it would entirely fail parsing if this is wrong

/* Constants:
 * LoRa pins
 * The number of bytes implied by each command byte
 * Motor control pins--motorA and motorB are the two sides of the H-bridge driver.
 * Motors armed?
 * Sensor pins; as implied.
 * Serial baud rate for the GPS
 * Serial baud rate for computer serial comms.
 * Verbosity - 0=no debug, 1=normal, 2=very verbose
 * Boolean sign for latitude (this software assumes longitude will always be positive)
 * Radius of location accuracy required at waypoints (m)
 * Tolerance on heading (governed by induced magnetometer inaccuracy)
 * Period between transmissions (during which the radio will be receiving)
 * Period between sensor readings
 */
//LoRa pins:
#define nss 20
#define dio0 7
#define dio5 16
#define rfm_rst 21

//radio command lengths
const byte cmd_lengths[8] = {0,8,2,1,1,2,2,2};

//motor pins
const byte motor_pins[] = {14,15,3,4};//a1,a2,b1,b2
boolean motors_armed = true;

//serial baud rates
#define gps_serial_baud_rate 9600
#define computer_serial_baud_rate 38400

#define verbosity 1 //verbosity for Serial debug



//timers
#define radio_transmit_period 900 //time in milliseconds
#define sensor_reading_period radio_transmit_period-180

/*
 * Global variables:
 * TinyGPSPlus object--receives and decodes GPS data.
 * SensLib - responsible for communication with sensors and decoding of their data
 * RFMLib - responsible for communication with the radio module. Abstraction layer.
 * LSM303 - magnetometer
 * Servo - parachute release
 * Next waypoints--capacity for five waypoints
 * Length of next waypoints array
 * Radio transmission timer.
 * Sensor reading timer
 * Packet received boolean
 * Manual motor control status
 * MS5637 read already?
 * Time to read?
 */
SensLib sns;//sensor object
RFMLib radio = RFMLib(nss,dio0,dio5,rfm_rst);//radio object
uint32_t radio_transmit_timer;
uint32_t sensor_read_timer;
boolean pkt_rx = false;
byte manual[] = {255,255};//assign to 255 to disable override, otherwise setting as normal.

boolean ms5637_read = false;
boolean read_sens = false;

/* Misc declarations/definitions
 * Prototype for assemblePacket statement--references apparently confuse the Arduino/Processing compiler, which is peculiar.
  (This problem has been reported by other users with different code.
 */
 
 void assemblePacket(RFMLib::Packet &pkt);
 void decodePacket(RFMLib::Packet pkt);
 
 
 void setup(){
   pinMode(3,OUTPUT);
   pinMode(4,OUTPUT);
  SPI.begin();//Join the SPI bus
  byte my_config[5] = {0x64,0x74,0xFA,0xAC,0xCD};//radio settings
  radio.configure(my_config);//Radio configuration
  
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  
  //Motor initialisation
  for(byte i = 0;i<5;i++){
    pinMode(motor_pins[i],OUTPUT);
    digitalWrite(motor_pins[i],LOW);
  }   
   radio_transmit_timer = millis();
}

void loop(){
  if(millis()-sensor_read_timer >= sensor_reading_period){
    read_sens = true; 
  }
  if(read_sens){
   if(ms5637_read){
    sns.pollHYT271();
    Serial.println(sns.humidity);
    read_sens = false;
    sensor_read_timer = millis(); 
    ms5637_read = false;
   }
   else{
    sns.pollMS5637();
    ms5637_read = true; 
   }
  }
  if(radio.rfm_done) finishRFM();
  
  Serial.println("--=-=-=-=--");
  Serial.println((millis()-radio_transmit_timer));
  Serial.println(radio.rfm_status);
  if((millis()-radio_transmit_timer) > radio_transmit_period && radio.rfm_status != 1)
    transmitTime();
}

void transmitTime(){
      Serial.println("time to tx");
      if(radio.rfm_status==2){
      RFMLib::Packet p;
     radio.endRX( p);
    }
    RFMLib::Packet p;
    assemblePacket(p);
    radio.beginTX(p); 
    attachInterrupt(7,RFMISR,RISING);
    radio_transmit_timer = millis();
    sensor_read_timer = millis();
}

void finishRFM(){
  switch(radio.rfm_status){
  case 1:
     #if verbosity != 0
     Serial.println("Ending transmission.");
     #endif
     radio.endTX();
     #if verbosity != 0
     Serial.println("Beginning reception.");
     #endif
     radio.beginRX();
     radio.rfm_done = false;
         attachInterrupt(7,RFMISR,RISING);
     break;
     case 2:
     #if verbosity != 0
     Serial.println("Ending reception.");
     #endif 
     RFMLib::Packet rx;
     radio.endRX(rx);
     decodePacket(rx);
     break;
   }

}


void RFMISR(){
 radio.rfm_done = true; 
}

void decodePacket(RFMLib::Packet pkt){
  byte i = 0;
  #if verbosity != 0
  Serial.println("Packet to be decoded: ");
  Serial.print("len = ");Serial.println(pkt.len);
       for(int k = 0;k<pkt.len;k++)Serial.println(pkt.data[k]);
  #endif

//  if(pkt.crc){
  while(i < pkt.len){

   switch(pkt.data[i]){
    case 0://general status OK - 1 byte
    #if verbosity > 0
     Serial.println("OK");
     #endif
     break;
    break;
    case 4://arm the motors - 2 bytes
      if (pkt.data[i+1]==255)
      motors_armed =true;
      else motors_armed = false;
      i++;
      #if verbosity > 0
      Serial.println("Motors armed.");
      #endif
    break;
   } 
   i++;
  }

  
  
}

void assemblePacket(RFMLib::Packet &pkt){
  //round the pressure and shave a decimal place off to fit it into 16 bits
  //saving two bytes of valuable bandwidth
  int32_t pr_calc = sns.pressure;
  Serial.println(sns.humidity);
  byte round_byte = ((pr_calc % 10)>4)?1:0;
  pr_calc /= 10;
  pr_calc += (int16_t) round_byte;
    #if verbosity > 0
  Serial.println((int16_t)pr_calc);
  #endif
  uint16_t small_pressure = (uint16_t) pr_calc;
  pkt.data[0] = (byte)(small_pressure >> 8);//pressure
  pkt.data[1] = small_pressure & 255;
  
  //HYT271 temp
  pkt.data[2] = (byte)(sns.external_temperature>>8);
  pkt.data[3] = sns.external_temperature & 255;
  
  //MS5637 temp
  pkt.data[4] = (byte)(sns.internal_temperature>>8);
  pkt.data[5] = sns.internal_temperature & 255;
  
  //humidity
  pkt.data[6] = (byte)(sns.humidity >> 8);
  pkt.data[7] = sns.humidity & 255;
 

  //set length
  pkt.len = 21;
  
  //ir data append here
}
