// Self Stabilizing Controllable Laser Mount
// Amy Pickens, Nate Honold, Tucker Kirven

#include <stdio.h>
#include <pthread.h>

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

// global mode to be set by thread3 and read by thread1
enum mode deviceMode;

static void *thread1function(void *arg) { return (void *)0; }
static void *thread2function(void *arg) { return (void *)0; }

int main() {

  pthread_t thread1, thread2;
  pthread_attr_t myattr;

  pthread_attr_init(&myattr);
  
  //thread1 gets MPU values and calculates desired servo position
  pthread_create(&thread1, &myattr, thread1function, (void *)0);
  //thread2 gets desired servo positions and sets servos
  pthread_create(&thread2, &myattr, thread2function, (void *)0);
  pthread_attr_destroy(&myattr);

  
  pthread_join(thread1, 0);
  pthread_join(thread2, 0);
}
