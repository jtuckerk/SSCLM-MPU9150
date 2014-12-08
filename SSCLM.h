#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "MPUfiles/I2Cdev.h"
#include "MPUfiles/MPU6050.h"
#include "MPUfiles/MPU6050_6Axis_MotionApps20.h"
#include <pthread.h>
#include <wiringPi.h>

// struct to hold rotation in degrees about X, Y and Z axis
struct XYZposition {
  int x;
  int y;
  int z;
};

// enum for the mode the device is in
enum mode { MODE_CONTROLLABLE, MODE_STABILIZE, MODE_COMBINED };

//===================================================================================//
//FUNCTION DECLARATIONS
//===================================================================================//
static void *getPosition(void *arg);
static void *setPosition(void *arg);
void initMPU(MPU6050 mpu);
void calculateServoPos(struct XYZposition *base, struct XYZposition *controller,
                       mode deviceMode);
bool getXYZ(MPU6050 *mpu, struct XYZposition *pos);
void setServo(int servoNum, int position);
void userModeControl();
void lights();
void setOffset(float base, float controller);
float waitStabalize(MPU6050 *mpu);

//===================================================================================//
//GLOBAL VARIABLES
//===================================================================================//

// global to hold the positions the servo should be in
// set by thread1 and read by thread2
// Karsai wants the value passed to thread2 to be a
// change in the current position - not an absolute value
struct XYZposition servoPositions;

// Stores the position for the arm to maintain during stabilize and combined
// modes
struct XYZposition lockPosition;

// global mode to be set by thread3 and read by thread1
enum mode deviceMode;

// Stores the x-axis offset between the controller and the base MPU
float baseOffset, controllerOffset;

// set to true if the position expected is not achievable by the
// servos
bool XinBounds = true;
bool YinBounds = true;
bool ZinBounds = true;

//
MPU6050 baseMPU(MPU6050_ADDRESS_AD0_HIGH), controlMPU;

// MPU device control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3];        // [yaw, pitch, roll]

//Protects the global variable holding the positions of the servo 
pthread_mutex_t servoPosMutex;
pthread_mutex_t servoCondMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t servoCond = PTHREAD_COND_INITIALIZER;
bool servoPosUpdated = false;

//Driver file for servo control using ServoBlaster Raspberry Pi driver
std::ofstream servoDriverFile;

#define SERVO_MIN 20
#define SERVO_MAX (180 - SERVO_MIN)
#define PI 3.14159
int servos[3] = {0, 1, 2};

#define BUTTON1 2 // WiringPi pin numbers
#define BUTTON2 3
#define BUTTON3 4
#define LED 16
