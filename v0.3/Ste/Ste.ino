#include <Wire.h>

/* I2C Addresses for IMUs */

#define IMU_B 0x68
#define IMU_T 0x69

#define ACCEL_XOUT_H 0x3B

/* Global Variables */

//Data Buffer
uint8_t Data[14];

// For IMU_B
double accelX_B,accelY_B,accelZ_B;
double gyroX_B, gyroY_B, gyroZ_B;
uint16_t temp_B;

// For IMU_T
double accelX_T,accelY_T,accelZ_T;
double gyroX_T, gyroY_T, gyroZ_T;
uint16_t temp_T;


void setup(){
    Serial.begin(115200);
    Wire.begin();

    unsigned int counter = 0;
    double mean_accx,mean_accy,mean_accz = 0;
    double var_accx,var_accy,var_accz = 0;

    while(true){
        readRegisters();
        counter++;
        mean_accx += accelX_B;
        mean_accy += accelY_B;
        mean_accz += accelZ_B;
//        Serial.println(mean_accy);
        if(counter >= 6000){
            mean_accx /= counter;
            mean_accy /= counter;
            mean_accz /= counter;
            break;
        }
    }
    counter = 0;
    while(true){
        readRegisters();
        counter++;
        var_accx += sq(accelX_B-mean_accx);
        var_accy += sq(accelY_B-mean_accy);
        var_accz += sq(accelZ_B-mean_accz);
//        Serial.print(String(accelY_B) + "\t" + String(var_accy)+ "\n");
        Serial.print(String(var_accx)+"\n");
        if(counter >= 6000){
            var_accx /= counter;
            var_accy /= counter;
            var_accz /= counter;
            break;
        }
    }
    Serial.print(String(counter)+"  ");
    Serial.print(String(mean_accx,3)+"  ");
    Serial.print(String(mean_accy,3)+"  ");
    Serial.println(mean_accz,3);

    Serial.print(String(var_accx,6)+" ");
    Serial.print(String(var_accy,6)+" ");
    Serial.print(String(var_accz,6)+" \n");
    

}

void loop(){

    readRegisters();
    

    // Serial.print(String(time)+"  ");
    // Serial.print(String(accelX_B,4)+"  ");
    // Serial.print(String(accelY_B,4)+"  ");
    // Serial.print(String(accelZ_B,4)+"  \n");
    // Serial.print(String(temp_B)+"  ");
    // Serial.print(String(gyroX_B,4)+"  ");
    // Serial.print(String(gyroY_B,4)+"  ");
    // Serial.print(String(gyroZ_B,4)+"\n");
    
    

}

void i2cRead(int IMU, uint8_t Reg, uint8_t data[], int nbytes){
    Wire.beginTransmission(IMU);
    Wire.write(Reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU,nbytes,true);
    for(uint8_t i=0;i<nbytes;i++){
        if(Wire.available()){
            data[i] = Wire.read();
        }
        
    }
}

void readRegisters(){
    i2cRead(IMU_B,ACCEL_XOUT_H,Data,14);

    accelX_B = (double)(Data[0] << 8 | Data[1]) / 16384;
    accelY_B = (double)(Data[2] << 8 | Data[3]) / 16384;
    accelZ_B = (double)(Data[4] << 8 | Data[5]) / 16384;    
    temp_B = (double)(Data[6] << 8 | Data[7]) / 340.00 + 36.53;    
    gyroX_B = (double)(Data[8] << 8 | Data[9]) / 131;
    gyroY_B = (double)(Data[10] << 8 | Data[11]) / 131;
    gyroZ_B = (double)(Data[12] << 8 | Data[13]) / 131;
}
