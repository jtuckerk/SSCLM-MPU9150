// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

#include "SSCLM.h"

static void *getPosition(void *arg) {

  struct XYZposition basePosition, controllerPosition;
  while (1) {
  
    if(deviceMode == MODE_CONTROLLABLE || deviceMode == MODE_COMBINED)
      getXYZ(&controlMPU, &controllerPosition);

    // calculate neccesary servo position and writes to the servoPositions
    // global variable

    pthread_cond_signal( &servoCond );
  }
  return (void *)0;
}

int main() {

  initMPU(controlMPU);
  usleep(100000); //@@

  // this usually takes about 15 seconds
  float controller = waitStabalize(&controlMPU);
 
  
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

  pthread_attr_destroy(&myattr);

  pthread_join(thread1, 0);

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
