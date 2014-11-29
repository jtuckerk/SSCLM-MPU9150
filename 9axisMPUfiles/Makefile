all: MPU9150_AHRS

MPU = MPUfiles

HDRS = helper_3dmath.h I2Cdev.h MPU6050_9Axis_MotionApps41.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o 
DMP_OBJS = MPU9150_AHRS.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

MPU9150_AHRS: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm -lpthread -lrt 

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS)  MPU9150_AHRS
