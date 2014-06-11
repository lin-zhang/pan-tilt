/*
 * A fblock for servo motor control.
 *
 */

/* #define DEBUG 1 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "ubx.h"
#include "servo_motor.h"

UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
/* declare and initialize a microblx type. This will be registered /
 * deregistered in the module init / cleanup at the end of this
 * file.
 *
 * Include regular header file and it's char array representation
 * (used for luajit reflection, logging, etc.)
 */

#include "types/servo_motor_driver_config.h"
#include "types/servo_motor_driver_config.h.hexarr"

#define MAX_NUM_MOTORS 8
/* declare the type and give the char array type representation as the type private_data */
ubx_type_t servo_motor_driver_config_type = def_struct_type(struct servo_motor_driver_config, &servo_motor_driver_config_h);

/* function block meta-data
 * used by higher level functions.
 */

char servo_motor_driver_meta[] =
	"{ doc='function block for FPGA AXI Communiation',"
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
ubx_config_t servo_motor_driver_config[] = {
	{ .name="axi_port_config", .type_name = "struct servo_motor_driver_config" },
	{ NULL },
};

/* Ports
 */
ubx_port_t servo_motor_driver_ports[] = {
	{ .name="p_data_in", .in_type_name="unsigned int", .in_data_len=8},
	{ .name="p_data_out", .out_type_name="unsigned int", .out_data_len=8},
	{ NULL },
};

/* block local info
 *
 * This struct holds the information needed by the hook functions
 * below.
 */
struct servo_motor_driver_info {
	int fd;
	unsigned int *p;
	int baseAddr;
	int nMotors;
	int usPwmPeriod;
	int usConst;
	int ZeroDegree;
	char runMode;
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
def_read_arr_fun(read_uint, unsigned int, 8)
def_write_arr_fun(write_uint, unsigned int, 8)
/**
 * servo_motor_driver_init - block init function.
 *
 * for RT blocks: any memory should be allocated here.
 *
 * @param b
 *
 * @return Ok if 0,
 */

static int servo_motor_driver_init(ubx_block_t *b)
{
	int ret=0;
	int i=0;
	int pwmCounter=0;
	DBG(" ");
	if ((b->private_data = calloc(1, sizeof(struct servo_motor_driver_info)))==NULL) {
		ERR("Failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
        struct servo_motor_driver_info* inf = (struct servo_motor_driver_info*) b->private_data;

	unsigned int clen;
	struct servo_motor_driver_config* servo_motor_driver_conf;

	/* get and store servo_motor_driver_config */
	servo_motor_driver_conf = (struct servo_motor_driver_config*) ubx_config_get_data_ptr(b, "axi_port_config", &clen); 
	inf->baseAddr = servo_motor_driver_conf->baseAddr; // Base address of PWM Config, 0x43C00000 
	inf->nMotors = servo_motor_driver_conf->nMotors; //number of motors to be used, the PWM config support up to 8 PWM generators. 
	inf->usPwmPeriod = servo_motor_driver_conf->usPwmPeriod; //PWM period in microsecond
	inf->usConst = servo_motor_driver_conf->usConst; //number of clock cycles for 1 microsecond. The FPGA clock takes 10ns, therefore usCount should be 100.
	inf->ZeroDegree = servo_motor_driver_conf->ZeroDegree;//Pulse width in microsecond for 0 degree position.

	/* using memory map to access PWM config */
        inf->fd=open("/dev/mem", O_RDWR);
        if(inf->fd == -1) {
        printf("Err: cannot open /dev/mem\n");
        return -1;
        }
        inf->p = (unsigned int *)mmap(0, 65536, PROT_READ|PROT_WRITE, MAP_SHARED, inf->fd, inf->baseAddr);
	if (inf->p == (unsigned int*)-1) {
               printf("Err: cannot access axi comm block controller!\n");
               return -1;
       	}
	
	/*Set PWM signal period of N motors in unit of clock cycle */
	pwmCounter = inf->usPwmPeriod * inf->usConst;
        for (i=0;i<inf->nMotors;i++){
                *(inf->p+i*2)=pwmCounter;
        }
	
	/*Set positions of the N motors to 0 degree*/
        for (i=0;i<inf->nMotors;i++){
                *(inf->p+i*2+1)=inf->ZeroDegree * inf->usConst;
        }
	
	/*Disable the motors that are not used, Nmax-N*/
        for (i=inf->nMotors;i<MAX_NUM_MOTORS;i++){
                *(inf->p+i*2)=0;
        }

 out:
	return ret;
}

/**
 * servo_motor_driver_cleanup - cleanup block.
 *
 * for RT blocks: free all memory here
 *
 * @param b
 */
static void servo_motor_driver_cleanup(ubx_block_t *b)
{
	DBG(" ");
	free(b->private_data);
}

/**
 * servo_motor_driver_start - start the servo_motor_driver block.
 *
 * @param b
 *
 * @return 0 if Ok, if non-zero block will not be started.
 */
static int servo_motor_driver_start(ubx_block_t *b)
{
	DBG("in");
	return 0; /* Ok */
}

/**
 * servo_motor_driver_step - this function implements the main functionality of the
 * block. Ports are read and written here.
 *
 * @param b
 */
static void servo_motor_driver_step(ubx_block_t *b) {
	int i;
	unsigned int data_in[MAX_NUM_MOTORS];
	unsigned int data_out[MAX_NUM_MOTORS];
	unsigned int *tmp;
 	struct servo_motor_driver_info* inf;

	inf=(struct servo_motor_driver_info*) b->private_data;

	ubx_port_t* servo_motor_in_port = ubx_port_get(b, "p_data_in");
	ubx_port_t* servo_motor_out_port = ubx_port_get(b, "p_data_out");

	//read_uint(servo_motor_in_port, (unsigned int*)data_in);
	read_uint(servo_motor_in_port, &data_in);
        tmp=inf->p;
        for(i=0;i<inf->nMotors;i++){
                *(tmp+2*i+1)=data_in[i];
        }

	//write_uint(servo_motor_out_port, &data_out);

	tmp=inf->p;
	for(i=0;i<inf->nMotors;i++){
		data_out[i]=*(tmp+2*i+1);
	}	
	//printf("\n");
	
	write_uint(servo_motor_out_port, &data_out);
}


/* put everything together
 *
 */
ubx_block_t servo_motor_driver_comp = {
	.name = "servo_motor_driver/servo_motor_driver",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = servo_motor_driver_meta,
	.configs = servo_motor_driver_config,
	.ports = servo_motor_driver_ports,

	/* ops */
	.init = servo_motor_driver_init,
	.start = servo_motor_driver_start,
	.step = servo_motor_driver_step,
	.cleanup = servo_motor_driver_cleanup,
};

/**
 * servo_motor_driver_module_init - initialize module
 *
 * here types and blocks are registered.
 *
 * @param ni
 *
 * @return 0 if OK, non-zero otherwise (this will prevent the loading of the module).
 */
static int servo_motor_driver_module_init(ubx_node_info_t* ni)
{
	ubx_type_register(ni, &servo_motor_driver_config_type);
	return ubx_block_register(ni, &servo_motor_driver_comp);
}

/**
 * servo_motor_driver_module_cleanup - de
 *
 * unregister blocks.
 *
 * @param ni
 */
static void servo_motor_driver_module_cleanup(ubx_node_info_t *ni)
{
	ubx_type_unregister(ni, "struct servo_motor_driver_config");
	ubx_block_unregister(ni, "servo_motor_driver/servo_motor_driver");
}

/* declare the module init and cleanup function */
UBX_MODULE_INIT(servo_motor_driver_module_init)
UBX_MODULE_CLEANUP(servo_motor_driver_module_cleanup)
