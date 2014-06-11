#define DEBUG 1 

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "ubx.h"

#include "types/imu_balance_config.h"
#include "types/imu_balance_config.h.hexarr"


UBX_MODULE_LICENSE_SPDX(GPL-2.0+)

#define MAX_NUM_MOTORS 8
#define BUFF_LEN 25
#define BUFF_PWM_LEN 16

#define MA_FILTER 1
#ifdef MA_FILTER
float IMU_buffer[BUFF_LEN][3];
int buffer_i;

unsigned int buff_arr_pwm[MAX_NUM_MOTORS][BUFF_PWM_LEN];
unsigned int out_avg_arr_pwm[MAX_NUM_MOTORS];
int buffer_pwm_i=0;
#endif

unsigned int in_arr_pwm[MAX_NUM_MOTORS]={0,0,0,0,0,0,0,0};
unsigned int out_arr_pwm[MAX_NUM_MOTORS]={0,0,0,0,0,0,0,0};

#define MIN_DEGREE1 180
#define MAX_DEGREE1 360
#define MIN_DEGREE2 180
#define MAX_DEGREE2 360
#define COE_SLOPE1 -934.44
#define COE_TRANS1 396500
#define COE_SLOPE2 934.44
#define COE_TRANS2 -108100
/* declare the type and give the char array type representation as the type private_data */
ubx_type_t imu_balance_config_type = def_struct_type(struct imu_balance_config, &imu_balance_config_h);

/* function block meta-data
 * used by higher level functions.
 */
char imu_balance_meta[] =
	"{ doc='A simple demo to link IMU block and PWM block',"
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
ubx_config_t imu_balance_config[] = {
	{ .name="min_max_config", .type_name = "struct imu_balance_config" }, //the config of this block is not used.
	{ NULL },
};

/* Ports
 */
ubx_port_t imu_balance_ports[] = {
	{ .name="in_port_imu", .in_type_name="float", .in_data_len=3},
	{ .name="in_port_pwm_sig", .in_type_name="unsigned int", .in_data_len=8},
	{ .name="in_port_random", .in_type_name="unsigned int"},
	{ .name="out_port_pwm_sig", .out_type_name="unsigned int", .out_data_len=8},
	{ NULL },
};

/* block local info
 *
 * This struct holds the information needed by the hook functions
 * below.
 */
struct imu_balance_info {
	float imu_val; //the imu_balance_info is not used in this example.
};

/* convenience functions to read/write from the ports these fill a
 * ubx_data_t, and call port->[read|write](&data). These introduce
 * some type safety.
 */
def_read_arr_fun(read_arr_float, float, 3)
def_read_arr_fun(read_arr_uint8, unsigned int, 8)
def_read_fun(read_random_uint, unsigned int)
def_write_arr_fun(write_arr_uint8, unsigned int, 8)
/**
 * imu_balance_init - block init function.
 *
 * for RT blocks: any memory should be allocated here.
 *
 * @param b
 *
 * @return Ok if 0,
 */
static int imu_balance_init(ubx_block_t *b)
{
	int ret=0;

	DBG(" ");
	if ((b->private_data = calloc(1, sizeof(struct imu_balance_info)))==NULL) {
		ERR("Failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
 out:
	return ret;
}

/**
 * imu_balance_cleanup - cleanup block.
 *
 * for RT blocks: free all memory here
 *
 * @param b
 */
static void imu_balance_cleanup(ubx_block_t *b)
{
	DBG(" ");
	free(b->private_data);
}

/**
 * imu_balance_start - start the imu_balance block.
 *
 * @param b
 *
 * @return 0 if Ok, if non-zero block will not be started.
 */
static int imu_balance_start(ubx_block_t *b)
{
	DBG(" ");
	return 0; /* Ok */
}

/**
 * imu_balance_step - this function implements the main functionality of the
 * block. Ports are read and written here.
 *
 * @param b
 */
static void imu_balance_step(ubx_block_t *b) {
	float in_arr_imu[3];
	float f_measured_data[3];
	unsigned int random;
	int ret;
        /*Define ports:
        * Input port: IMU measurements
        * Input port: Servo motor positions from PWM block
        * Output port: Set points for servo motors to PWM block
        */
	ubx_port_t* in_port_imu = ubx_port_get(b, "in_port_imu");
        ubx_port_t* in_port_pwm_sig = ubx_port_get(b, "in_port_pwm_sig");
        ubx_port_t* in_port_random_sig = ubx_port_get(b, "in_port_random");
        ubx_port_t* out_port_pwm_sig = ubx_port_get(b, "out_port_pwm_sig");

	read_arr_float(in_port_imu, &in_arr_imu);
	read_arr_uint8(in_port_pwm_sig,&in_arr_pwm);
	
	
	ret=read_random_uint(in_port_random_sig, &random);
	if(ret>0){
	in_arr_imu[1]+=(float)random/10-50;
	DBG("Random number generated is: %u, %f", random, in_arr_imu[1]);
	}

#ifdef MA_FILTER
	/*Apply moving average filter to remove measurement noise*/
	int i;
        if(buffer_i<BUFF_LEN-1) buffer_i++; else buffer_i=0;
        IMU_buffer[buffer_i][0]=in_arr_imu[0];
	IMU_buffer[buffer_i][1]=in_arr_imu[1];
        IMU_buffer[buffer_i][2]=in_arr_imu[2];

        for(i=0;i<BUFF_LEN;i++){
	       f_measured_data[0]+=IMU_buffer[i][0];
	       f_measured_data[1]+=IMU_buffer[i][1];
 	       f_measured_data[2]+=IMU_buffer[i][2];
        }
        f_measured_data[0]/=BUFF_LEN;	
	f_measured_data[1]/=BUFF_LEN;
	f_measured_data[2]/=BUFF_LEN;
#else
	f_measured_data[0]=in_arr_imu[0];
	f_measured_data[1]=in_arr_imu[1];
	f_measured_data[2]=in_arr_imu[2];
#endif


	/*Code to link IMU orientation to 2-DOF gadget*/
	//result of x-y angle f_measured_data[0] is not need in the demo.
	if(f_measured_data[1]>MIN_DEGREE1 && f_measured_data[1]<MAX_DEGREE1){	
		out_arr_pwm[0]=(unsigned int)(f_measured_data[1]*(COE_SLOPE1)+COE_TRANS1);	
	}
	else 
		out_arr_pwm[0]=in_arr_pwm[0];

	if(f_measured_data[2]>MIN_DEGREE2 &&f_measured_data[2]<MAX_DEGREE2){	
		out_arr_pwm[1]=(unsigned int)(f_measured_data[2]*COE_SLOPE2+COE_TRANS2);
	}
	else 
		out_arr_pwm[1]=in_arr_pwm[1];

	DBG("%f, %f, %u, %u\n", f_measured_data[1], f_measured_data[2], out_arr_pwm[0], out_arr_pwm[1]);
	
#ifdef MA_FILTER
	/*Apply moving average filter to smooth motor trajectory*/
	int j;
        if(buffer_pwm_i<BUFF_PWM_LEN-1) buffer_pwm_i++; else buffer_pwm_i=0;
        for(i=0;i<MAX_NUM_MOTORS;i++)
                        buff_arr_pwm[i][buffer_pwm_i]=out_arr_pwm[i];
        for(i=0;i<MAX_NUM_MOTORS;i++){
                for(j=0;j<BUFF_PWM_LEN;j++)
                        out_arr_pwm[i]+=buff_arr_pwm[i][j];
        }
        for(i=0;i<MAX_NUM_MOTORS;i++)
                out_arr_pwm[i]/=(BUFF_PWM_LEN+1);
#endif

	/*Send motor set points to output port for PWM config*/
	write_arr_uint8(out_port_pwm_sig, &out_arr_pwm);
	

}


/* put everything together
 *
 */
ubx_block_t imu_balance_comp = {
	.name = "imu_balance/imu_balance",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = imu_balance_meta,
	.configs = imu_balance_config,
	.ports = imu_balance_ports,

	/* ops */
	.init = imu_balance_init,
	.start = imu_balance_start,
	.step = imu_balance_step,
	.cleanup = imu_balance_cleanup,
};

/**
 * imu_balance_module_init - initialize module
 *
 * here types and blocks are registered.
 *
 * @param ni
 *
 * @return 0 if OK, non-zero otherwise (this will prevent the loading of the module).
 */
static int imu_balance_module_init(ubx_node_info_t* ni)
{
	ubx_type_register(ni, &imu_balance_config_type);
	return ubx_block_register(ni, &imu_balance_comp);
}

/**
 * imu_balance_module_cleanup - de
 *
 * unregister blocks.
 *
 * @param ni
 */
static void imu_balance_module_cleanup(ubx_node_info_t *ni)
{
	ubx_type_unregister(ni, "struct imu_balance_config");
	ubx_block_unregister(ni, "imu_balance/imu_balance");
}

/* declare the module init and cleanup function */
UBX_MODULE_INIT(imu_balance_module_init)
UBX_MODULE_CLEANUP(imu_balance_module_cleanup)
