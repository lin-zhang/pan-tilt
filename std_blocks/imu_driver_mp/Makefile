ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
INCLUDE_DIR=$(ROOT_DIR)/src/

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

imu_driver.so: imu_driver.o imu.o $(INCLUDE_DIR)/libubx.so 
	${CC} $(CFLAGS_SHARED) -o imu_driver.so imu_driver.o imu.o $(INCLUDE_DIR)/libubx.so

imu_driver.o: imu_driver.c $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) imu_driver.c

imu.o: imu.c $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) imu.c


clean:
	rm -f *.o *.so *~ core $(HEXARRS)
