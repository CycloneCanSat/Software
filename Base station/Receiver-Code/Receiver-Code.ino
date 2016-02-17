#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(20,7,16,21);
#define nss 20
void setup(){
  SPI.begin();
  Serial.begin(38400);
  byte my_config[6] = {0x44,0x84,0x88,0xAC,0xCD, 0x08};
  radio.configure(my_config);
}

void loop(){
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
    int pressure = (int)rx.data[4] * 256 + (int)rx.data[5];
    int externalTemp = (int)rx.data[6] * 256 + (int)rx.data[7];
    int humidity = (int)rx.data[8] * 256 + (int)rx.data[9];
    int seconds = (int)rx.data[10] * 256 + (int)rx.data[11];
    int hours = seconds / 3600;
    seconds%=3600;
    int minutes = seconds / 60;
    seconds %= 60;
    int sats = (int)rx.data[12];
    int longitude = (int)rx.data[13] * 16777216 + (int)rx.data[14] * 65536 + (int)rx.data[15] * 256 + (int)rx.data[16];
    int latitude = (int)rx.data[17] * 16777216 + (int)rx.data[18] * 65536 + (int)rx.data[19] * 256 + (int)rx.data[20];
    int alt = (int)rx.data[21] * 256 + (int)rx.data[22];
    int heading = (int)rx.data[23];
    int pitch = (int)rx.data[24];
    word roll = rx.data[25];

    Serial.print("Sample Number\t");
    Serial.println(sampleNumber);
    Serial.print("Internal Temp\t");
    Serial.println(internalTemp);
     Serial.print("Pressure\t");
    Serial.println(pressure);
     Serial.print("External Temp\t");
    Serial.println(externalTemp);
     Serial.print("Humidity\t");
    Serial.println(humidity);
     Serial.print("Time\t");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.println(seconds);
     Serial.print("Satellites\t");
    Serial.println(sats);
     Serial.print("Longitude\t");
    Serial.println(longitude);
     Serial.print("Latitude\t");
    Serial.println(latitude);
     Serial.print("Altitude\t");
    Serial.println(alt);
     Serial.print("Heading\t");
    Serial.println(heading);
     Serial.print("Pitch\t");
    Serial.println(pitch);
     Serial.print("Roll\t");
    Serial.println(roll);
    Serial.println();
    Serial.println();
    Serial.println();
    int snr = rx.snr;
    int rssi = rx.rssi;
    Serial.print("Signal to Noise Ratio\t");
    Serial.println(snr);
    Serial.print("Received Signal Strength Indication\t");
    Serial.println(rssi);
  }
   Serial.println();
  }
  
}

void RFMISR(){
 radio.rfm_done = true; 
}


