
all: SSCLM

HDRS = ./MPUfiles/helper_3dmath.h ./MPUfiles/I2Cdev.h ./MPUfiles/MPU6050_6Axis_MotionApps20.h ./MPUfiles/MPU6050.h 
CMN_OBJS = I2Cdev.o MPU6050.o
DMP_OBJS = SSCLM.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

SSCLM: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS)  SSCLM
