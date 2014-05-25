#include <iostream>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"

struct XYZposition {
  int x;
  int y;
  int z;
};

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16
    aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float
    ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void getXYZ(MPU6050 *mpu, struct XYZposition *pos) {
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu->resetFIFO();
    printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (fifoCount >= 42) {
    // read a packet from FIFO
    mpu->getFIFOBytes(fifoBuffer, packetSize);

    // display Euler angles in degrees
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
    //    printf("ypr  %7.2f %7.2f %7.2f    \n",90+ ypr[0] * 180 / M_PI,
    //    90+ypr[1] * 180 / M_PI,90+ ypr[2] * 180 / M_PI);

    pos->x = (ypr[0] * 180 / M_PI) + 90;
    pos->y = (ypr[1] * 180 / M_PI) + 90;
    pos->z = (ypr[2] * 180 / M_PI) + 90;
  }
}

void initMPU(MPU6050 mpu) {
  // initialize device
  printf("Initializing I2C devices...\n");
  mpu.initialize();

  // verify connection
  printf("Testing device connections...\n");
  printf(mpu.testConnection() ? "MPU6050 connection successful\n"
                              : "MPU6050 connection failed\n");

  // load and configure the DMP
  printf("Initializing DMP...\n");
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    printf("Enabling DMP...\n");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external
    // interrupt 0)..."));
    // attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
    printf("DMP ready!\n");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    printf("DMP Initialization failed (code %d)\n", devStatus);
  }
}
int main (int argc, char *argv[]){
struct XYZposition servoPositions;
 MPU6050 mpu, mpu2(MPU6050_ADDRESS_AD0_HIGH);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

   initMPU(mpu);

   std::cout << "mpu has address "<< mpu.devAddr<< std::endl;
   std::cout<<std::flush;
   usleep(100000);
printf(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
   usleep(100000);
initMPU(mpu2);
 std::cout << "mpu2 has address "<< mpu2.devAddr<< std::endl;
   usleep(100000);
printf(mpu2.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 while(1){
   getXYZ(&mpu, &servoPositions);
   usleep(10000);
   std::cout <<"mpu1:\t"<< servoPositions.x << "\t" << servoPositions.y << "\t" << servoPositions.z;
getXYZ(&mpu2, &servoPositions);
   std::cout <<"\t\t\tmpu2:\t"<< servoPositions.x << "\t" << servoPositions.y << "\t" << servoPositions.z << std::endl;
   

   /*
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    std::cout <<"Accel:\t"<< ax << " " << ay << " " << az << std::endl;
    std::cout <<"Gravity:\t"<< gx << " " << gy << " " << gz << std::endl;
    std::cout <<"magno:\t"<< mx << " " << my << " " << mz << std::endl;
   */
 }
  return 0;
}
