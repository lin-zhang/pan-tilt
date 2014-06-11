/*
 * A fblock to access IMU config.
 *
 * This is to be a well (over) documented block to serve as a good
 * example.
 */

/* #define DEBUG 1 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "ubx.h"
#include "imu.h"

UBX_MODULE_LICENSE_SPDX(GPL-2.0+)

/* declare and initialize a microblx type. This will be registered /
 * deregistered in the module init / cleanup at the end of this
 * file.
 *
 * Include regular header file and it's char array representation
 * (used for luajit reflection, logging, etc.)
 */
#include "types/imu_driver_config.h"
#include "types/imu_driver_config.h.hexarr"

/* declare the type and give the char array type representation as the type private_data */
ubx_type_t imu_driver_config_type = def_struct_type(struct imu_driver_config, &imu_driver_config_h);

/* function block meta-data
 * used by higher level functions.
 */
char imu_driver_meta[] =
	"{ doc='function block for FPGA based Sparkfun 9DoF SensorStick',"
	"  license='LGPL',"
	"  real-time=true,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */

/*.name is set as "sensor_config" instead of "imu_driver_config" */
ubx_config_t imu_driver_config[] = {
	{ .name="sensor_config", .type_name = "struct imu_driver_config" },
	{ NULL },
};

/* Ports
 */
ubx_port_t imu_driver_ports[] = {
	{ .name="result_arr", .out_type_name="float", .out_data_len=3 },
	{ NULL },
};

/* block local info
 *
 * This struct holds the information needed by the hook functions
 * below.
 */
struct imu_driver_info {
	unsigned int sensor_base_addr;
	int sensor_slave_addr;
	int sensor_data_addr;	
	int fd;
	unsigned int *p;
	float res_imu[3];
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
//def_write_fun(write_float, float)
def_write_arr_fun(write_arr_float, float, 3)


/**
 * imu_driver_init - block init function.
 *
 * for RT blocks: any memory should be allocated here.
 *
 * @param b
 *
 * @return Ok if 0,
 */
static int imu_driver_init(ubx_block_t *b)
{
	int ret=0;

	DBG(" ");
	if ((b->private_data = calloc(1, sizeof(struct imu_driver_info)))==NULL) {
		ERR("Failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
        struct imu_driver_info* inf = (struct imu_driver_info*) b->private_data;
	
	unsigned int clen;
	struct imu_driver_config* imu_driver_conf;

        imu_driver_conf = (struct imu_driver_config*) ubx_config_get_data_ptr(b, "sensor_config", &clen);

        inf->sensor_base_addr = imu_driver_conf->sensor_base_addr;
        inf->sensor_slave_addr = imu_driver_conf->sensor_slave_addr;
        inf->sensor_data_addr = imu_driver_conf->sensor_data_addr;


        /* using memory map to access IMU config */	
        inf->fd=open("/dev/mem", O_RDWR);
        if(inf->fd == -1) {
        printf("Err: cannot open /dev/mem\n");
        return -1;
        }
        inf->p = (unsigned int *)mmap(0, 65536, PROT_READ|PROT_WRITE, MAP_SHARED, inf->fd, inf->sensor_base_addr);
	if (inf->p == (unsigned int*)-1) {
               printf("Err: cannot access axi comm block controller!\n");
               return -1;
       	}

	/*Accelerometer initialization*/
        init_AXDL345(inf->p);

 out:
	return ret;
}

/**
 * imu_driver_cleanup - cleanup block.
 *
 * for RT blocks: free all memory here
 *
 * @param b
 */
static void imu_driver_cleanup(ubx_block_t *b)
{
	DBG(" ");
	free(b->private_data);
}

/**
 * imu_driver_start - start the imu_driver block.
 *
 * @param b
 *
 * @return 0 if Ok, if non-zero block will not be started.
 */
static int imu_driver_start(ubx_block_t *b)
{
	DBG("in");
	return 0; /* Ok */
}

/**
 * imu_driver_step - this function implements the main functionality of the
 * block. Ports are read and written here.
 *
 * @param b
 */
static void imu_driver_step(ubx_block_t *b) {
	float result[3];
 	struct imu_driver_info* inf;

	inf=(struct imu_driver_info*) b->private_data;

	ubx_port_t* imu_arr_value_port = ubx_port_get(b, "result_arr");

	/*x-y, x-z and y-z angles are read through "result" array*/
	read_AXDL345(inf->p, ACL_SLAVE_ADDR, ACL_FIRST_DATA_REG, result);
	write_arr_float(imu_arr_value_port, &result);
	
}


/* put everything together
 *
 */
ubx_block_t imu_driver_comp = {
	.name = "imu_driver/imu_driver",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = imu_driver_meta,
	.configs = imu_driver_config,
	.ports = imu_driver_ports,

	/* ops */
	.init = imu_driver_init,
	.start = imu_driver_start,
	.step = imu_driver_step,
	.cleanup = imu_driver_cleanup,
};

/**
 * imu_driver_module_init - initialize module
 *
 * here types and blocks are registered.
 *
 * @param ni
 *
 * @return 0 if OK, non-zero otherwise (this will prevent the loading of the module).
 */
static int imu_driver_module_init(ubx_node_info_t* ni)
{
	ubx_type_register(ni, &imu_driver_config_type);
	return ubx_block_register(ni, &imu_driver_comp);
}

/**
 * imu_driver_module_cleanup - de
 *
 * unregister blocks.
 *
 * @param ni
 */
static void imu_driver_module_cleanup(ubx_node_info_t *ni)
{
	ubx_type_unregister(ni, "struct imu_driver_config");
	ubx_block_unregister(ni, "imu_driver/imu_driver");
}

/* declare the module init and cleanup function */
UBX_MODULE_INIT(imu_driver_module_init)
UBX_MODULE_CLEANUP(imu_driver_module_cleanup)
