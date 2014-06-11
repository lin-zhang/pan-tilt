ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
INCLUDE_DIR=$(ROOT_DIR)/src/

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

servo_motor_driver.so: servo_motor_driver.o servo_motor.o $(INCLUDE_DIR)/libubx.so 
	${CC} $(CFLAGS_SHARED) -o servo_motor_driver.so servo_motor_driver.o servo_motor.o $(INCLUDE_DIR)/libubx.so

servo_motor_driver.o: servo_motor_driver.c $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) servo_motor_driver.c

servo_motor.o: servo_motor.c $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) servo_motor.c


clean:
	rm -f *.o *.so *~ core $(HEXARRS)
