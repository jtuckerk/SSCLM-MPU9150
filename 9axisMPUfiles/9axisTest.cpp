#include <iostream>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"




int main (int argc, char *argv[]){

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

    accelgyro.initialize();

printf(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 while(1){
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    std::cout <<"Accel:\t"<< ax << " " << ay << " " << az << std::endl;
    std::cout <<"Gravity:\t"<< gx << " " << gy << " " << gz << std::endl;
    std::cout <<"magno:\t"<< mx << " " << my << " " << mz << std::endl;
 }
  return 0;
}
