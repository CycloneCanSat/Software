#include "sensor_library.h"
//must include wire library in main sketch

SensLib::SensLib(){
}
void SensLib::initialise(){
  Wire.beginTransmission(ms_addr);
  Wire.write(0x1E);
  Wire.endTransmission();

  for(int i = 0;i<7;i++){
    Wire.beginTransmission(ms_addr);
    int reg = 0xA0;
    reg += (2*i);//each calibration variable is 2 bytes; we only need to call the MSB address.
    Wire.write(reg);//PROM registers range from 0xA0 to 0xAE
    Wire.endTransmission();
    Wire.requestFrom(ms_addr,2);
    long timer = millis();
    while(!Wire.available()){
      if((millis() - timer)>1000)
        break;//abandon setup
    }
    calib[i]=(Wire.read()<<8) | Wire.read();//store in calibration value array
  }
}

void SensLib::pollMS5637(){
    uint32_t d1,d2;
    //read data
    Wire.beginTransmission(ms_addr);

    Wire.write(0x4A);//d1 convert
    Wire.endTransmission();
    delay(25);//typ time is 16.44ms for conversion at 8192 bit oversampling ratio - would guess that this is more or less constant, but doesn't hurt to allow an extra 9ms!
    Wire.beginTransmission(ms_addr);
    Wire.write(0x00);//ADC read
    Wire.endTransmission();
    Wire.requestFrom(ms_addr,3);//24 bits
    while(!Wire.available());
    d1 = (Wire.read()<<16) | (Wire.read()<<8) | Wire.read();//read in all three bits and get D1

    Wire.beginTransmission(ms_addr);//now for d2
    Wire.write(0x5A);//d2 convert
    Wire.endTransmission();
    delay(25);//typ time is 16.44ms for conversion at 8192 bit oversampling ratio - would guess that this is more or less constant, but doesn't hurt to allow an extra 9ms!
    Wire.beginTransmission(ms_addr);
    Wire.write(0x00);//ADC read
    Wire.endTransmission();
    Wire.requestFrom(ms_addr,3);//24 bits
    while(!Wire.available());

    d2 = (Wire.read()<<16) | (Wire.read()<<8) | Wire.read();

    //Now for conversion

    //calculate temperature
    int32_t dt = (int32_t)(d2- (int32_t)calib[5] * 256);
    int32_t temp =(int32_t)(2000+(int32_t)(dt*(int32_t)calib[6])/(int32_t)8388608);
    //temp in 1/100ths of a degC

    //Calculate pressure variables
    int64_t off = (int64_t)((int64_t)calib[2] * (int64_t)131072 + ((int64_t)calib[4]*(int64_t)dt) /(int64_t)64);
    int64_t sens =(int64_t)((int64_t)calib[1] * (int64_t)65536 + ((int64_t)calib[3] * (int64_t)dt)/(int64_t)128);
    int64_t sens2 = 0;
    int64_t off2 = 0;
    int64_t t2;
    if(temp>2000){//"second order temperature compensation"
      t2 = (int64_t)((int64_t)5 * (int64_t)((int64_t)dt*(int64_t)dt) / (int64_t)274877906944);
      //high temp
    }
    else{
      //low temp
      t2 = (int64_t)((int64_t)3 * (int64_t)((int64_t)dt*(int64_t)dt) / (int64_t)8589934592);
      off2 = (int64_t)temp - (int64_t)2000;
      off2 = (int64_t)(off2 * off2);
      sens2 = off2;
      off2 = (int64_t)((int64_t)61 * off2 / 16);
      sens2 = (int64_t)((int64_t)(29 * sens2) / 16);
    }
    //won't be less than -15degC with electronics, so don't need to account for this case.
    temp = (int32_t)((int64_t)temp - t2);//apply compensation
    off = off-off2;
    sens = sens-sens2;

    pressure = (int32_t)((d1 * sens / 2097152 - off) / 32768);//calculate final pressure
    internal_temperature = temp;
}

void SensLib::pollHYT271(){
  byte data[4];
  Wire.beginTransmission(hyt_addr);
  Wire.endTransmission();//send measurement request
  delay(80);//spec 60ms.
  Wire.requestFrom(hyt_addr,4);//V Important - we don't wait for wire available,
  //in case this causes a hang.
  for(byte i = 0;i<4;i++){
    data[i] = Wire.read();//read the expected five bytes into an array.
  }

  Wire.endTransmission();


  int rawH = ((data[0] << 8) & 0x3FFF) | data[1];
  int rawT = data[2] << 6;
  rawT = rawT | (data[3] >> 2);
  external_temperature = (int16_t)((((float)(rawT) * 165.0 / 16384.0) - 40.0)*100);
  humidity = (int16_t)(((float)rawH * 100.0 / 16384.0)*100);
}
