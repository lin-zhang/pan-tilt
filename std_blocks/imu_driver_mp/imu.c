#include "imu.h"

void Xil_Out32(unsigned int *address, u32 offset_in_hex, u32 data){
        *(address+(offset_in_hex>>2))=data;
}

void Xil_Out8(unsigned int *address, u32 offset_in_hex, u8 data){
        *(u8 *)(address+(offset_in_hex>>2))=data;
}


u32 Xil_In32(unsigned int *address, u32 offset_in_hex){
        return *(address+(offset_in_hex>>2));
}

void init_AXDL345(unsigned int *address){
		int i;
		printf("AXDL345 accelerometer selected ..\n\r");
		/* Issue Soft Reset command */
		XIo_Out32(address,0x40,0x0000000A);

		/* Delay for reset completion */
		usleep(DELAY_VAL);

		/* Reset Transmit FIFOs */
		XIo_Out32(address,0x100,0x00000002);
                usleep(DELAY_VAL);

		/* Delay for reset completion */
		iic_write(address, ACL_SLAVE_ADDR,0x31,0x00); 	// 0x00 => 2g, 4mg/LSB

                usleep(DELAY_VAL);
									// 0x01 => 4g, 8mg/LSB
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x2c,0x08);
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x2d,0x08);
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x2e,0x00);
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x1e,0x00);		// 0xF9 = dec -6, for 2g mode
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x1f,0x00);
		usleep(DELAY_VAL);
		iic_write(address, ACL_SLAVE_ADDR,0x20,0x00);
		usleep(DELAY_VAL);
		XIo_Out32(address, 0x40, 0x0000000A);
		XIo_Out32(address, 0x100, 0x00000002);

		/* Initialise Master Tx FIFO Empty, Rx FIFO Full and Tx Error interrupts */
		XIo_Out32(address,0x28,0x0000000E);
                usleep(DELAY_VAL);

		/* Write to GPO register to set GPO bit */
		XIo_Out32(address,0x124,0x00000001);
                usleep(DELAY_VAL);

		/* Initialise IIC controllers */
		/* Enable IIC Controller */
		XIo_Out32(address,0x100,0x00000001);
                usleep(DELAY_VAL);

		/* Write to GPO register to set GPO bit */
		XIo_Out32(address,0x124,0x00000000);
                usleep(DELAY_VAL);

		/* Read Status register */
		printf("\r\n-- Reading Status Register --");
		i=XIo_In32(address,0x104);
		printf("\r\n-- Status Register is (first) %x --\n",i);

		/* Write to GPO register to set GPO bit */
		XIo_Out32(address,0x124,0x00000001);
                usleep(DELAY_VAL);

};

void read_AXDL345(unsigned int *address, u8 slaveAddr,u8 regAddr, float result[3]){
	u32 x_l,x_h;
	u32 y_l,y_h;
	u32 z_l,z_h;
	signed short x,y,z;
	float xy,xz,yz;
	XIo_Out32(address, 0x108,0x00000100 | slaveAddr);
	XIo_Out32(address, 0x108,0x00000000 | regAddr);
	XIo_Out32(address, 0x108,0x00000100 | slaveAddr | 0x00000001);
	XIo_Out32(address, 0x108,0x00000206);
	XIo_Out32(address, 0x124,0x00000000);
   	/* Set Master Mode and Transmit */
	Xil_Out8(address, 0x100, 0x0D);
	x_l = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	x_h = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	y_l = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	y_h = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	z_l = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	z_h = XIo_In32(address, 0x10C);
	usleep(DELAY_VAL2);
	x= (x_h<<8 | x_l) & 0xffff;
	y= (y_h<<8 | y_l) & 0xffff;
	z= (z_h<<8 | z_l) & 0xffff;
	xy=atan2((float)y,(float)x)*180/3.14159265+180;
        xz=atan2((float)z,(float)x)*180/3.14159265+180;
        yz=atan2((float)z,(float)y)*180/3.14159265+180;
	result[0]=xy;
	result[1]=xz;
	result[2]=yz;
}

void iic_write(unsigned int *address, u8 slaveAddr, u8 regAddr, u8 data){
        Xil_Out32(address, 0x28, 0x0000000E);
	Xil_Out32(address, 0x124, 0x00000001);
        Xil_Out32(address, 0x100, 0x00000001);
        Xil_Out32(address, 0x124, 0x00000000);

	Xil_Out32(address, 0x124, 0x00000001);

        Xil_Out32(address, 0x108, 0x00000100 | slaveAddr);
        Xil_Out32(address, 0x108, 0x00000000 | regAddr);
        Xil_Out32(address, 0x108, 0x00000200 | data);
}


u32 iic_read(unsigned int * address, u8 slaveAddr,u8 regAddr){
	u32 data;
	XIo_Out32(address,0x108,0x00000100 | slaveAddr);
	XIo_Out32(address,0x108,0x00000000 | regAddr);
	XIo_Out32(address,0x108,0x00000100 | slaveAddr | 0x00000001);
	XIo_Out32(address,0x108,0x00000201);
	XIo_Out32(address,0x124,0x00000000);
	/* Set Master Mode and Transmit */
	Xil_Out8(address,0x100,0x0D);
	data = XIo_In32(address,0x10C);
	return data;
}
