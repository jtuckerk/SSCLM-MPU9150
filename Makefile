all: SSCLM_read

MPU = MPUfiles

HDRS = $(MPU)/helper_3dmath.h $(MPU)/I2Cdev.h $(MPU)/MPU6050_6Axis_MotionApps20.h $(MPU)/MPU6050.h
CMN_OBJS = $(MPU)/I2Cdev.o $(MPU)/MPU6050.o 
DMP_OBJS = SSCLM_read.o

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

SSCLM_read: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm -lpthread -lrt

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS)  SSCLM_read
