#include <iostream>
#include <math.h>
#include "I2Cdev.h"
#inlcude "MPU6050_9Axis_MotionApps41.h"




int main (int argc, char *argv[]){

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  return 0;
}
