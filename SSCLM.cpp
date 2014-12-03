// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

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
enum mode { MODE_CONTROLLABLE, MODE_STABILIZE, MODE_COMBINED};

// forward declarations
void initMPU(MPU6050 mpu);
void calculateServoPos(struct XYZposition *base, struct XYZposition *controller,
                       mode deviceMode);
void getXYZ(MPU6050 *mpu, struct XYZposition *pos);
void setServo(SERVO servoNum, int position);
void userModeControl();
void lights();
void setOffset(float base, float controller);
float waitStabalize(MPU6050 *mpu);

// global to hold the positions the servo should be in
// set by thread1 and read by thread2
// Karsai wants the value passed to thread2 to be a
// change in the current position - not an absolute value
struct XYZposition servoPositions;

// Stores the position for the arm to maintain during stabilize and combined
// modes
struct XYZposition lockPosition;

// Stores the x-axis offset between the controller and the base
float baseOffset, controllerOffset;

// set to true if the position expected is not achievable by the
// servos
bool XinBounds = true;
bool YinBounds = true;
bool ZinBounds = true;

// need to change address of one or both MPUs
// they will both have default address out of the box
// changing them once should be saved on the device
MPU6050 baseMPU(MPU6050_ADDRESS_AD0_HIGH), controlMPU;

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
Quaternion q;   // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] 

#define SERVO_MIN 20
#define SERVO_MAX (180 - SERVO_MIN)
#define PI 3.14159
int servos[3] = {0, 1, 2};

pthread_mutex_t servoPosMutex;
std::ofstream servoDriverFile;

#define BUTTON1 2 // WiringPi pin numbers
#define BUTTON2 3
#define BUTTON3 4
#define LED 16

static void *getPosition(void *arg) {

  struct XYZposition basePosition, controllerPosition;
  while (1) {
    if (deviceMode != MODE_CALIBRATE &&
        (deviceMode == MODE_STABILIZE || deviceMode == MODE_COMBINED))
      getXYZ(&baseMPU, &basePosition);

    usleep(1000);
    if (deviceMode != MODE_CALIBRATE &&
        (deviceMode == MODE_CONTROLLABLE || deviceMode == MODE_COMBINED))
      getXYZ(&controlMPU, &controllerPosition);

    // calculate neccesary servo position and writes to the servoPositions
    // global variable
    calculateServoPos(&basePosition, &controllerPosition, deviceMode);
  }
  return (void *)0;
}
static void *setPosition(void *arg) {

  int servoPosX, servoPosY, servoPosZ;

  while (1) {
    pthread_mutex_lock(&servoPosMutex);
    servoPosX = servoPositions.x;
    servoPosY = servoPositions.y;
    servoPosZ = servoPositions.z;
    pthread_mutex_unlock(&servoPosMutex);

    setServo(servos[0], servoPosX);
    setServo(servos[1], servoPosY);
    setServo(servos[2], servoPosZ);
  }

  return (void *)0;
}

int main() {

  initMPU(controlMPU);
  usleep(100000);
  initMPU(baseMPU);
  usleep(100000);

  wiringPiSetup();
  // Set I/O pin directions
  pinMode(LED, OUTPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);

  float controller = waitStabalize(&controlMPU);
  float base = waitStabalize(&baseMPU);
 
  setOffset(base, controller);

  deviceMode = MODE_CONTROLLABLE;
  // Opens device driver file that controls servo motors
  servoDriverFile.open("/dev/servoblaster");
  
  pthread_mutexattr_t mutexattr;
  pthread_mutexattr_init(&mutexattr);
  pthread_mutex_init(&servoPosMutex, &mutexattr);
  pthread_mutexattr_destroy(&mutexattr);

  pthread_t thread1, thread2;
  pthread_attr_t myattr;

  pthread_attr_init(&myattr);

  // thread1 gets MPU values and calculates desired servo position
  pthread_create(&thread1, &myattr, getPosition, (void *)0);
  // thread2 gets desired servo positions and sets servos
  pthread_create(&thread2, &myattr, setPosition, (void *)0);
  pthread_attr_destroy(&myattr);

  // continuously checks for mode changes and out of bounds errors
  while (true) {
    userModeControl();
  }

  pthread_join(thread1, 0);
  pthread_join(thread2, 0);
  return 0;

  // initialize device
  printf("Initializing I2C devices...\n");

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
bool getXYZ(MPU6050 *mpu, struct XYZposition *pos) {
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return false;
  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu->resetFIFO();
    // printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (fifoCount >= 42) {
    // read a packet from FIFO
    mpu->getFIFOBytes(fifoBuffer, packetSize);

    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

    pos->x = (ypr[0] * 180 / M_PI + 90);
    pos->y = (ypr[1] * 180 / M_PI + 90);
    pos->z = (ypr[2] * 180 / M_PI + 90);
    return true;
  }
}

bool boundServo(int *pos) {
  if (*pos > SERVO_MAX) {
    *pos = SERVO_MAX;
    return false;
  } else if (*pos < SERVO_MIN) {
    *pos = SERVO_MIN;
    return false;
  } else {
    return true;
  }
}

void calculateServoPos(struct XYZposition *base, struct XYZposition *controller,
                       mode deviceMode) {

  int cx = controller->x;
  int cy = controller->y;
  int cz = controller->z;
  int bx = base->x;
  int by = base->y;
  int bz = base->z;

  int x, y, z;

  switch (deviceMode) {

  case MODE_CALIBRATE: // Don't move servos
    break;

  case MODE_CONTROLLABLE:

    x = cx - controllerOffset;
    y = cy;
    z = 180 - cz;
    // printf("MODE1: %d %d %d\n", x, y, z);

    break;

  case MODE_STABILIZE:

    x = 90 + (lockPosition.x - bx) -
        baseOffset; // lockPosition.x - (bx - lockPosition.x)
    y = 90 + (lockPosition.y - by);
    z = 180 - (90 + (lockPosition.z - bz));
    // printf("MODE2: %d %d %d\n", x, y, z);

    break;

  case MODE_COMBINED:

    x = 90 + ((cx - controllerOffset) - bx) -
        baseOffset; // + offset; // cx - (bx - cx)
    y = 90 + (cy - by);
    z = 180 - (90 + (cz - bz));
    // printf("MODE3: %d %d %d\n", x, y, z);

    break;

  default:
    x = 90;
    y = 90;
    z = 90;
    break;
  }

  XinBounds = boundServo(&x);
  YinBounds = boundServo(&y);
  ZinBounds = boundServo(&z);
  lights();
  std::cout << "BASE: " << bx << " " << by << " " << bz << "\t";
  std::cout << "CONTROLLER: " << cx << " " << cy << " " << cz << "\t";
  std::cout << "SERVO: "
            << " " << x << " " << y << " " << z << " " << std::endl;

  pthread_mutex_lock(&servoPosMutex);
  servoPositions.x = x;
  servoPositions.y = y;
  servoPositions.z = z;
  pthread_mutex_unlock(&servoPosMutex);
}

void setServo(SERVO servoNum, int position) {
  //ServoBlaster driver expects input in range 0%-100% 
  position = (int)(position / 1.8);
  servoDriverFile << servoNum << "=" << position << "%" << std::endl;
}

// 3 buttons- 1 for each mode
// button push changes mode
// called in main method()

void userModeControl() {

  // not sure what all we can use
  // http://wiringpi.com/
  // https://projects.drogon.net/raspberry-pi/gpio-examples/tux-crossing/software/

  usleep(100000); // need to test to find correct number

  // mode 1- controllable
  if (digitalRead(BUTTON1) == HIGH) {
    // if button1 pushed (and released)
    deviceMode = MODE_CONTROLLABLE;
    printf("Button 1 pushed\n");
  }

  // mode 2- self-stabilize
  else if (digitalRead(BUTTON2) == HIGH) {
    // if button2 pushed (and released)

    pthread_mutex_lock(&servoPosMutex);
    lockPosition.x = servoPositions.x;
    lockPosition.y = servoPositions.y;
    lockPosition.z = servoPositions.z;
    pthread_mutex_unlock(&servoPosMutex);

    deviceMode = MODE_STABILIZE;
    printf("Button 2 pushed\n");
    printf("MODE2: %d %d %d\n", lockPosition.x, lockPosition.y, lockPosition.z);
  }

  // mode 3- combined
  else if (digitalRead(BUTTON3) == HIGH) {
    // if button3 pushed (and released)

    deviceMode = MODE_COMBINED;
    printf("Button 3 pushed: Combined Mode\n");
  }
}

void setOffset(float base, float controller) {

  baseOffset = base - 90;
  controllerOffset = controller - 90 - baseOffset;
}

// lights up lights when servo is expected to do something it cannot do
// uses wiringPi
// called in main()
void lights() {

  if (!XinBounds || !YinBounds || !ZinBounds) {
    // write 1 to turn on LED
    digitalWrite(LED, 1);

  } else {
    // write 0 to turn off LED
    digitalWrite(LED, 0);
  }
}
float waitStabalize(MPU6050 *mpu) {
  bool stable = false;
  digitalWrite(LED, 1);
  XYZposition pos;
  float startyaw, endyaw;
  while (!stable) {
    for (int i = 0; i < 100;) {
      usleep(1000);
      
      if(getXYZ(mpu, &pos)){
	i++;
      
	if (i == 25)
	  digitalWrite(LED, 0);
	if (i == 75)
	  digitalWrite(LED, 1);
	if (i == 1) {
	  startyaw = 90 + pos.x * 180 / M_PI;
	  std::cout << "start Yaw: " << startyaw << "\t";
	}
	if (i == 99) {
	  endyaw = 90 + pos.x * 180 / M_PI;
	  std::cout << "End Yaw: " << endyaw << std::endl;
	}
        
      }
    }
    float diff = endyaw - startyaw;
    if (diff < .01 && diff > -.01)
      stable = true;
  }
  return startyaw;
}
