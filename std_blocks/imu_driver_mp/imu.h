#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <math.h>

//#define IIC_BASEADDR 0x41600000
//#define ROBOT_BASEADDR 0x43C00000


#define ACL_SLAVE_ADDR 0xA6
#define ACL_FIRST_DATA_REG 0x32

#define GYRO_SLAVE_ADDR 0xD0
#define GYRO_LSB_PER_DEG_PER_SEC 14.375
#define GYRO_FIRST_DATA_REG 0x1D

#define COMPASS_SLAVE_ADDR 0x3C
#define COMPASS_FIRST_DATA_REG 0x03
#define DELAY_VAL_COMPASS 6000

#define DELAY_VAL 10000
#define DELAY_VAL2 100
#define u32 unsigned int
#define u8 unsigned char
#define XIo_Out32 Xil_Out32
#define XIo_In32 Xil_In32

#define Ximu_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define Ximu_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

//typedef uint8_t u8;
//typedef uint16_t u16;
//typedef uint32_t u32;

void Xil_Out32(unsigned int *address, u32 offset_in_hex, u32 data);
void Xil_Out8(unsigned int *address, u32 offset_in_hex, u8 data);
u32 Xil_In32(unsigned int *address, u32 offset_in_hex);
void init_AXDL345(unsigned int *address);
void read_AXDL345(unsigned int *address, u8 slaveAddr,u8 regAddr, float result[3]);
void iic_write(unsigned int *address, u8 slaveAddr, u8 regAddr, u8 data);
u32 iic_read(unsigned int * address, u8 slaveAddr,u8 regAddr);
