// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

#include "SSCLM.h"

static void *getPosition(void *arg) {

  struct XYZposition basePosition, controllerPosition;
  while (1) {
    if (deviceMode == MODE_STABILIZE || deviceMode == MODE_COMBINED)
      getXYZ(&baseMPU, &basePosition);

  
    if(deviceMode == MODE_CONTROLLABLE || deviceMode == MODE_COMBINED)
      getXYZ(&controlMPU, &controllerPosition);

    // calculate neccesary servo position and writes to the servoPositions
    // global variable
    calculateServoPos(&basePosition, &controllerPosition, deviceMode);
    servoPosUpdated = true;
    pthread_cond_signal( &servoCond );
  }
  return (void *)0;
}
static void *setPosition(void *arg) {

  int servoPosX, servoPosY, servoPosZ;

  while (1) {
    pthread_mutex_lock(&servoCondMutex);
    while(!servoPosUpdated)
      pthread_cond_wait( &servoCond, &servoCondMutex);
    pthread_mutex_unlock(&servoCondMutex);
    
    pthread_mutex_lock(&servoPosMutex);
    servoPosX = servoPositions.x;
    servoPosY = servoPositions.y;
    servoPosZ = servoPositions.z;
    pthread_mutex_unlock(&servoPosMutex);

    setServo(servos[0], servoPosX);
    setServo(servos[1], servoPosY);
    setServo(servos[2], servoPosZ);
    servoPosUpdated = false;
    
    //the below code was used to check how often the
    //servos were being updated in each mode
    //we found the combined mode which reads both MPU's
    //to take about 5 times as long as the other 2 modes
    /*
    clock_gettime(CLOCK_REALTIME, &currentTime);
    printf("Nano seconds from last set: %d\n",
		currentTime.tv_nsec-lastTime.tv_nsec);
		lastTime.tv_nsec = currentTime.tv_nsec;*/
  }
  return (void *)0;
}

int main() {

  initMPU(controlMPU);
  usleep(100000); //@@
  initMPU(baseMPU);
  usleep(100000); //@@

  //initializes WiringPi IO library
  wiringPiSetup();
  // Set I/O pin directions
  pinMode(LED, OUTPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);

  // waits for sensor values to settle
  // this usually takes about 15 seconds
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
}
// MPU initialization code - borrowed from a demo 
// provided by Jeff Rowberg, MPU library author.
// Initializes the device and its digital motion processing
// (dmp) and verifies that it is succesful
void initMPU(MPU6050 mpu) {
  // initialize device
  std::cout<<"Initializing I2C devices...\n";

  // verify connection
  std::cout<<"Testing device connections...\n";
  printf(mpu.testConnection() ? "MPU6050 connection successful\n"
                              : "MPU6050 connection failed\n");

  // load and configure the DMP
  std::cout<<"Initializing DMP...\n";
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    std::cout<<"Enabling DMP...\n";
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external
    // interrupt 0)..."));
    // attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
    std::cout<<"DMP ready!\n";
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    std::cout<<"DMP Initialization failed (code << devStatus"<<")\n";
  }
}
// Reads the FIFO buffer on the MPU9150 and uses the MPU library
// to interpret those values and output the orientation in a
// yaw, pitch, roll format
// the values are converted to degrees and offset to match
// orient the servos correctly
bool getXYZ(MPU6050 *mpu, struct XYZposition *pos) {
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return false;
  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu->resetFIFO();
    // DEBUG_PRINT("FIFO overflow!\n";

    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (fifoCount >= 42) {
    // read a packet from FIFO
    mpu->getFIFOBytes(fifoBuffer, packetSize);

    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

    pos->x = (ypr[0] * 180 / M_PI + ZEROPOS);
    pos->y = (ypr[1] * 180 / M_PI + ZEROPOS);
    pos->z = (ypr[2] * 180 / M_PI + ZEROPOS);
    return true;
  }
}
// keeps the servo from shaking by trying to
// to go beyond its maximum or minimum position
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

  case MODE_CONTROLLABLE:
    //mimics the controller 
    x = cx - controllerOffset;
    y = cy;
    z = 180 - cz;
    break;

  case MODE_STABILIZE:
    // offsets to maintain the last position before mode switch
    x = ZEROPOS + (lockPosition.x - bx) -
        baseOffset; // lockPosition.x - (bx - lockPosition.x)
    y = ZEROPOS + (lockPosition.y - by);
    z = 180 - (ZEROPOS + (lockPosition.z - bz)); //@@
    break;

  case MODE_COMBINED:
    // mimics the controller no matter the base position
    x = ZEROPOS + ((cx - controllerOffset) - bx) -
        baseOffset; // + offset; // cx - (bx - cx)
    y = ZEROPOS + (cy - by);
    z = 180 - (ZEROPOS + (cz - bz));

    break;

  default:
    x = ZEROPOS;
    y = ZEROPOS;
    z = ZEROPOS;
    break;
  }

  // Ensures the servos are not set beyond their capabilities
  // sets a flag if any servo is expected to go out of range
  XinBounds = boundServo(&x);
  YinBounds = boundServo(&y);
  ZinBounds = boundServo(&z);

  PRINT_DEBUG4("BASE: %d %d %d \t", bx, by, bz);
  PRINT_DEBUG4("CONTROLLER: %d %d %d \t ", cx, cy, cz);
  PRINT_DEBUG4("SERVO: %d %d %d \n", x, y, z);
  
  // sets lights if any servo cannot reach the requested position 
  outOfBoundsLight();
  


  pthread_mutex_lock(&servoPosMutex);
  servoPositions.x = x;
  servoPositions.y = y;
  servoPositions.z = z;
  pthread_mutex_unlock(&servoPosMutex);
}

// Takes a servo number 0, 1, 2 - x, y, z respectively
// and sets them to a value between 0 and 100%
// our servos do not operate well beyond 20 and 80% so
// setServo will always recieve values within that range
void setServo(int servoNum, int position) {
  //ServoBlaster driver expects input in range 0%-100% 
  position = (int)(position / 1.8);
  servoDriverFile << servoNum << "=" << position << "%" << std::endl;
}

// 3 buttons- 1 for each mode
// button push changes mode
// called in main method()
void userModeControl() {

  usleep(100000); 

  // mode 1- controllable
  if (digitalRead(BUTTON1) == HIGH) {
    // if button1 pushed (and released)
    deviceMode = MODE_CONTROLLABLE;
    PRINT_DEBUG("Button 1 pushed\n");
  }

  // mode 2- self-stabilize
  else if (digitalRead(BUTTON2) == HIGH) {
    // if button2 pushed (and released)

    //sets to stabalize at the most recent position
    pthread_mutex_lock(&servoPosMutex);
    lockPosition.x = servoPositions.x;
    lockPosition.y = servoPositions.y;
    lockPosition.z = servoPositions.z;
    pthread_mutex_unlock(&servoPosMutex);

    deviceMode = MODE_STABILIZE;
    PRINT_DEBUG("Button 2 pushed\n");
    
  }

  // mode 3- combined
  else if (digitalRead(BUTTON3) == HIGH) {
    // if button3 pushed (and released)

    deviceMode = MODE_COMBINED;
    PRINT_DEBUG("Button 3 pushed: Combined Mode\n");
  }
}
// When the two MPU's are initialized facing the same direction
// their YAW or Z values may still be different, this sets the 
// base MPU to be the zeroPosition (mechanically fixed at that position)
// and zeroes the controller off of that position value.
void setOffset(float base, float controller) {

  baseOffset = base - ZEROPOS;
  controllerOffset = controller - ZEROPOS - baseOffset;
}

// lights up lights when servo is expected to do something it cannot do
// Called after servo position is bounded in calculateServoPos
void outOfBoundsLight() {

  if (!XinBounds || !YinBounds || !ZinBounds) {
    // write 1 to turn on LED
    digitalWrite(LED, 1);

  } else {
    // write 0 to turn off LED
    digitalWrite(LED, 0);
  }
}
// Upon initialization the MPU's yaw value can take anywhere
// between a few milliseconds and 15 seconds to stabalize 
// while still. This algorithm ensures that the position
// will no longer drift once the device is being used
// while this is taking place the output light blinks 
float waitStabalize(MPU6050 *mpu) {
  bool stable = false;
  digitalWrite(LED, 1);
  XYZposition pos;
  float startyaw, endyaw;
  while (!stable) {
    for (int i = 0; i < 100;) {
      usleep(1000);
      if (!dmpReady){
	i--;
	
      }else{
	// get current FIFO count
	fifoCount = mpu->getFIFOCount();

	if (fifoCount == 1024) {
	  // reset so we can continue cleanly
	  mpu->resetFIFO();
	  //printf("FIFO overflow!\n");

	  // otherwise, check for DMP data ready interrupt (this should happen
	  // frequently)
	} else if (fifoCount >= 42) {
	  // read a packet from FIFO
	  mpu->getFIFOBytes(fifoBuffer, packetSize);

	  // display Euler angles in degrees
	  mpu->dmpGetQuaternion(&q, fifoBuffer);
	  mpu->dmpGetGravity(&gravity, &q);
	  mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
	  printf("i: %d yaw %7.2f\n",i,ypr[0] * 180 / M_PI);
	  i++;
	}
	if (i == 25)
	  digitalWrite(LED, 0);
	if (i == 75)
	  digitalWrite(LED, 1);
	if (i == 1) {

	  startyaw = ZEROPOS+ypr[0] * 180 / M_PI;
	  std::cout << "start Yaw: " << startyaw << "\t";
	}
	if (i == 99) {
	  endyaw = ZEROPOS+ypr[0] * 180 / M_PI;
	  std::cout << "End Yaw: " << endyaw << std::endl;

	}
        
      }
    }
    //ensures minimal difference over a fixed period of time
    //this has been tested to indicate stabilization
    float diff = endyaw - startyaw;
    if (diff < .01 && diff > -.01)
      stable = true;
  }
  return endyaw;
}
