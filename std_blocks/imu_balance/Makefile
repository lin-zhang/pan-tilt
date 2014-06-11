ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
INCLUDE_DIR=$(ROOT_DIR)/src/

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

imu_balance.so: imu_balance.o $(INCLUDE_DIR)/libubx.so 
	${CC} $(CFLAGS_SHARED) -o imu_balance.so imu_balance.o $(INCLUDE_DIR)/libubx.so

imu_balance.o: imu_balance.c $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) imu_balance.c

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
