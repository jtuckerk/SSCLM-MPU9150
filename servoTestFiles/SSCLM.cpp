#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <pthread.h>
// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven



// struct to hold rotation in degrees about X, Y and Z axis
struct XYZposition {
  int x;
  int y;
  int z;
};

// enum for the mode the device is in
enum mode { STABALIZE, CONTROLLABLE, COMBINED_MODE };


// global to hold the positions the servo should be in
// set by thread1 and read by thread2
// Karsai wants the value passed to thread2 to be a
// change in the current position - not an absolute value
struct XYZposition servoPositions;

// need to change address of one or both MPUs
// they will both have default address out of the box
// changing them once should be saved on the device
MPU6050 controlMPU;

// global mode to be set by thread3 and read by thread1
enum mode deviceMode;

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
pthread_mutex_t XYZ_mutex;

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
void getXYZ(MPU6050 *mpu, struct XYZposition *pos) {

  fflush(stdout);
  // if programming failed, don't try to do anything
  if (!dmpReady){
    printf("dmp not ready");
    return;
  }
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
    //printf("ypr  %7.2f %7.2f %7.2f    \n", ypr[0] * 180 / M_PI,
    //ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
    fflush(stdout);

  pos->x = ypr[0] * 180 / M_PI;
  pos->y = ypr[1] * 180 / M_PI;
  pos->z = ypr[2] * 180 / M_PI;
  }
}
void calculateServoPos(struct XYZposition *base, struct XYZposition *controller,
                       mode deviceMode) {
 
  pthread_mutex_lock(&XYZ_mutex);
  servoPositions.x = controller->x+90;
  pthread_mutex_unlock(&XYZ_mutex);
  //printf( "x:%d\n",controller->z);
}

static void *thread1function(void *arg) {

  fflush(stdout);
  struct XYZposition basePosition, controllerPosition;
  while(1){
    //if (deviceMode == STABALIZE || deviceMode == COMBINED_MODE)
    //getXYZ(&baseMPU, &basePosition);

    //if (deviceMode == CONTROLLABLE || deviceMode == COMBINED_MODE)
    getXYZ(&controlMPU, &controllerPosition);

  
  // calculate neccesary servo position and writes to the servoPositions
  // global variable
  calculateServoPos(&basePosition, &controllerPosition, deviceMode);
  }
  return (void *)0;
}
std::ofstream myfile;

static void *thread2function(void *arg) {
  usleep(150000);
  
  fflush(stdout);
  while(1){
    int pos=90;
  pthread_mutex_lock(&XYZ_mutex);
  pos = servoPositions.x;
  pthread_mutex_unlock(&XYZ_mutex);

  if (pos>160)
    pos=160;
  if (pos<20)
    pos=20;
    
  double val = pos/180.0;
  val=val*100;
  pos = (int)val;
   myfile<<"0="<<pos<<"%"<<std::endl;
  std::cout<<"0="<<pos<<"%"<<std::endl;


  usleep(1500);
  //printf( "0=%d%%\n",pos);
  } 
  
return (void *)0; }


int main() {

  //initMPU(baseMPU);
  initMPU(controlMPU);
    usleep(10000);
  pthread_t thread1, thread2;
  pthread_attr_t myattr;
  deviceMode = CONTROLLABLE;
  pthread_attr_init(&myattr);
  printf("init done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  myfile.open ("/dev/servoblaster");
  printf("file open");
  // thread1 gets MPU values and calculates desired servo position
  pthread_create(&thread1, &myattr, thread1function, (void *)0);
  // thread2 gets desired servo positions and sets servos
  pthread_create(&thread2, &myattr, thread2function, (void *)0);
  pthread_attr_destroy(&myattr);
  printf("threads created");
  pthread_join(thread1, 0);
  pthread_join(thread2, 0);
  return 0;
}