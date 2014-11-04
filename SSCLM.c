// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// struct to hold rotation in degrees about X, Y and Z axis
struct XYZposition {
  int x;
  int y;
  int z;
} XYZpos;

// enum for the mode the device is in
enum mode { STABALIZE, CONTROLLABLE, COMBINED_MODE };

// MPU init forward declaration
void initMPUs();
void calculateServoPos(XYZpos *base, XYZpos *controller, mode deviceMode);

// global to hold the positions the servo should be in
// set by thread1 and read by thread2
// Karsai wants the value passed to thread2 to be a
// change in the current position - not an absolute value
XYZpos servoPositions;

// need to change address of one or both MPUs
// they will both have default address out of the box
// changing them once should be saved on the device
MPU6050 baseMPU, controlMPU;

// global mode to be set by thread3 and read by thread1
enum mode deviceMode;

static void *thread1function(void *arg) {

  XYZpos basePosition, controllerPosition;

  if (deviceMode == STABALIZE || deviceMode == COMBINED_MODE)
    basePosition = getXYZ(&baseMPU);

  if (deviceMode == CONTROLLABLE || deviceMode == COMBINED_MODE)
    controllerPosition = getXYZ(&controlMPU);
  
  //calculate neccesary servo position and writes to the servoPositions
  //global variable
  calculateServoPos(&basePosition, &controllerPosition, deviceMode);

  return (void *)0;
}
static void *thread2function(void *arg) { return (void *)0; }

int main() {

  initMPU(baseMPU);
  initMPU(controlMPU);

  pthread_t thread1, thread2;
  pthread_attr_t myattr;

  pthread_attr_init(&myattr);

  // thread1 gets MPU values and calculates desired servo position
  pthread_create(&thread1, &myattr, thread1function, (void *)0);
  // thread2 gets desired servo positions and sets servos
  pthread_create(&thread2, &myattr, thread2function, (void *)0);
  pthread_attr_destroy(&myattr);

  pthread_join(thread1, 0);
  pthread_join(thread2, 0);
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
void getXYZ(MPU6050 *mpu) {
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
    printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180 / M_PI,
           ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
  }
}
void calculateServoPos(XYZpos *base, XYZpos *controller, mode deviceMode) {}
