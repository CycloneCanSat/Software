/* (C) Team Impulse 2014
 * Made available under a modification of the MIT license - see repository root.
 * Sensor library; MS5637 and HYT-271.
 */

 #ifndef SensLib_h
 #define SensLib_h

 #include <Arduino.h>
 #include <Wire.h>
 #define ms_addr 0x76
 #define hyt_addr 0x28

 class SensLib {
 public:
   int16_t internal_temperature;
   int16_t external_temperature;
   int16_t humidity;
   int32_t pressure;
   SensLib();
   void pollMS5637();
   void pollHYT271();//consider breaking out into requesting measurement and reading+processing data
                    //because there is a 60ms delay during measurement at this OSR.
   void initialise();
private:
  uint16_t calib[7];

};

#endif
