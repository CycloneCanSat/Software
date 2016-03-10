#include <SPI.h>
#include<math.h>
#include <RFM98W_library.h>
#include <Wire.h>
#include <sensor_library.h>
#include<TinyGPS++.h>
#define gpsSerial Serial1
TinyGPSPlus gps;
RFMLib radio =RFMLib(20,7,16,21);
#define nss 20
SensLib sns; // Sensor Object
float qfe;
boolean msRead = false;
boolean readSens = false;
double oldmillis;
int readingPeriod;  
float groundpressure;
float groundtemp;
void setup(){
  gpsSerial.begin(4800);
  Wire.begin();//join the I2C bus
  sns.initialise();//initialise the sensors connected over I2C
  SPI.begin();
  Serial.begin(115200);
  byte my_config[6] = {0x62,0x74,0x88,0x84,0x7B, 0x08};
  radio.configure(my_config);
  delay(5000);
  Serial.println("Signal to Noise Ratio, Received Signal Strength Indication, Sample Number, Internal Temperature, Barometric Pressure, External Temperature, Humidity, Time, GPS Fix (No of Satellites), Longitude, Latitude, Altitude, Heading, Pitch, Roll, Agricultural Viability, Pressure (Atms), Altitude from Pressure Sensor, Dew Point, Ground Barometric Pressure, Ground Temperature");
  qfe = 1013.25;
  readingPeriod = 500;
  oldmillis = millis();
  groundpressure = (float)sns.pressure / 100.0;
  groundtemp = (float)sns.internal_temperature / 100.0; 
} 
void loop(){
  if (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }
  if ((millis() - oldmillis) > readingPeriod)
  {
    sns.pollHYT271();
    sns.pollMS5637();
    float groundpressure = (float)sns.pressure / 100.0;
    float groundtemp = (float)sns.internal_temperature / 100.0;
    oldmillis = millis();
   }
  if(Serial.available() > 0)
  {
    qfe = Serial.parseInt();
    if(qfe == 0)
    {
      sns.pollMS5637();
      qfe = sns.pressure / 100.0;
    }
  }
  if(radio.rfm_status == 0){
    radio.beginRX(); 
    attachInterrupt(7,RFMISR,RISING);
  }

  if(radio.rfm_done){ 
    RFMLib::Packet rx;
    radio.endRX(rx);
  if(rx.len ==  26)
  {
    int sampleNumber = (int)rx.data[0] * 256 + (int)rx.data[1];
    int internalTemp = (int)rx.data[2] * 256 + (int)rx.data[3];
    float itemp = (float)internalTemp / 100;
    int pressure = (int)rx.data[4] * 256 + (int)rx.data[5];
    float ipress = (float)pressure / 10;
    int externalTemp = (int)rx.data[6] * 256 + (int)rx.data[7];
    float etemp = (float)externalTemp / 100;
    int humidity = (int)rx.data[8] * 256 + (int)rx.data[9];
    float ihum = (float)humidity / 100;
    int seconds = (int)rx.data[10] * 256 + (int)rx.data[11];
    //for now
    //seconds += (256 * 256);
    /* 256 x 256 = 65536
     *  Max seconds value calculated = 24 * 3600 + 3600 + 60 = 86400 + 3600 + 60 = 90,060
     *  Hence seconds shouldn't fit in two bytes
     *  However, since max value for time without the overflow would be about 6pm, we don't think this is a problem
     *  Saving a precious byte is more important
     */
    int hours = seconds / 3600;
    seconds = seconds % 3600;
    int minutes = seconds / 60;
    seconds %= 60;
    int sats = (int)rx.data[12];
    int longitude = (int)rx.data[13] * 16777216 + (int)rx.data[14] * 65536 + (int)rx.data[15] * 256 + (int)rx.data[16];
    float ilong = (float)longitude/1000000000;
    int latitude = (int)rx.data[17] * 16777216 + (int)rx.data[18] * 65536 + (int)rx.data[19] * 256 + (int)rx.data[20];
    float ilat = (float)latitude / 10000000;
    int alt = (int)rx.data[21] * 256 + (int)rx.data[22];
    int heading = (int)rx.data[23];
    int pitch = (int)rx.data[24];
    int roll = rx.data[25];
    float gamma = log(ihum/100.0) + ((17.67 * etemp)/(243.5 + etemp));
    float dewPoint = ((243.5 * gamma)/(17.67 - gamma));
    // float dewPoint = (((log((ihum)/100)) + ((17.62 * etemp)/(243.12+etemp)))/(17.62 - (log((ihum)/(100)) + ((17.62*etemp)/(243.12+etemp)))));
    float epress = ipress / 1013.2501;
    float tempViability = 1-(pow((etemp-27),2))/(972);
    float presViability = (-(pow(epress, 2))+(3*epress)-1)/epress;
    float humViability = -1*pow((ihum/100),2)+((2*ihum)/100);
    float agriViability = tempViability * presViability * humViability;
    float h = ((etemp+273)/(-0.0065)) * (pow((ipress/qfe),((-8.31432 * -0.0065)/(9.80665*0.0289644)))-1);
    sns.pollHYT271();
    sns.pollMS5637();
    float groundpressure = (float)sns.pressure / 100.0;
    float groundtemp = (float)sns.internal_temperature / 100.0;
    
//    Serial.print("Sample Number\t");
//    Serial.println(sampleNumber);
//    Serial.print("Internal Temp\t");
//    Serial.println(itemp, 3);
//     Serial.print("Pressure\t");
//    Serial.println(ipress, 3);
//     Serial.print("External Temp\t");
//    Serial.println(etemp, 3);
//     Serial.print("Humidity\t");
//    Serial.println(ihum, 3);
//     Serial.print("Time\t");
//    Serial.print(hours);
//    Serial.print(":");
//    Serial.print(minutes);
//    Serial.print(":");
//    Serial.println(seconds);
//     Serial.print("Satellites\t");
//    Serial.println(sats);
//     Serial.print("Longitude\t");
//    Serial.println(ilong, 9);
//     Serial.print("Latitude\t");
//    Serial.println(ilat, 7);
//     Serial.print("Altitude\t");
//    Serial.println(alt);
//     Serial.print("Heading\t");
//    Serial.println(heading);
//     Serial.print("Pitch\t");
//    Serial.println(pitch);
//     Serial.print("Roll\t");
//    Serial.println(roll);
//    Serial.println();
//    Serial.println();
//    Serial.println();
    int snr = rx.snr;
    int rssi = rx.rssi;
//    Serial.print("Signal to Noise Ratio\t");
//    Serial.println(snr);
//    Serial.print("Received Signal Strength Indication\t");
//    Serial.println(rssi);
  Serial.print(snr);
  Serial.print(",");
  Serial.print(rssi);
  Serial.print(",");
  Serial.print(sampleNumber);
  Serial.print(",");
  Serial.print(itemp);
  Serial.print(",");
  Serial.print(ipress);
  Serial.print(",");
  Serial.print(etemp);
  Serial.print(",");
  Serial.print(ihum);
  Serial.print(",");
  Serial.print(hours);
  Serial.print(",");
  Serial.print(minutes);
  Serial.print(",");
  Serial.print(seconds);
  Serial.print(",");
  Serial.print(sats);
  Serial.print(",");
  Serial.print(ilong);
  Serial.print(",");
  Serial.print(ilat);
  Serial.print(",");
  Serial.print(alt);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(agriViability,10);
  Serial.print(",");
  Serial.print(epress, 10);
  Serial.print(",");
  Serial.print(h, 10);
  Serial.print(",");
  Serial.print(dewPoint, 10);
  Serial.print(",");
  Serial.print(groundpressure);
  Serial.print(",");
  Serial.print(groundtemp);
  Serial.print(",");
  Serial.print(gps.location.lat(), 10);
  Serial.print(",");
  Serial.print(gps.location.lng(),10);
  Serial.print(",");
  Serial.println(gps.altitude.meters(), 10);
  
  }

  }
  
}

void RFMISR(){
 radio.rfm_done = true; 
}


