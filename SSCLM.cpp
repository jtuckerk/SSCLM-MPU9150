// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

#include "SSCLM.h"

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
}
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
bool getXYZ(MPU6050 *mpu, struct XYZposition *pos) {
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return false;
  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu->resetFIFO();
    // std::cout<<"FIFO overflow!\n";

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
   

    break;

  case MODE_STABILIZE:

    x = 90 + (lockPosition.x - bx) -
        baseOffset; // lockPosition.x - (bx - lockPosition.x)
    y = 90 + (lockPosition.y - by);
    z = 180 - (90 + (lockPosition.z - bz));
   

    break;

  case MODE_COMBINED:

    x = 90 + ((cx - controllerOffset) - bx) -
        baseOffset; // + offset; // cx - (bx - cx)
    y = 90 + (cy - by);
    z = 180 - (90 + (cz - bz));
   

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
    std::cout<<"Button 1 pushed\n";
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
    std::cout<<"Button 2 pushed\n";
    
  }

  // mode 3- combined
  else if (digitalRead(BUTTON3) == HIGH) {
    // if button3 pushed (and released)

    deviceMode = MODE_COMBINED;
    std::cout<<"Button 3 pushed: Combined Mode\n";
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
