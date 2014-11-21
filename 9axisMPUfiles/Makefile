all: 9axisTest

MPU = MPUfiles

HDRS = helper_3dmath.h I2Cdev.h MPU6050_9Axis_MotionApps41.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o 
DMP_OBJS = 9axisTest.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

9axisTest: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm -lpthread 

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS)  9axisTest
