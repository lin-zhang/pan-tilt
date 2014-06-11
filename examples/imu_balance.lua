#!/usr/bin/env luajit

local ffi = require("ffi")
local ubx = require "ubx"
--local ubx_utils = require("ubx_utils")
local ts = tostring

ni=ubx.node_create("testnode")

ubx.load_module(ni, "std_types/stdtypes/stdtypes.so")
ubx.load_module(ni, "std_types/testtypes/testtypes.so")
ubx.load_module(ni, "std_blocks/imu_driver_mp/imu_driver.so")
ubx.load_module(ni, "std_blocks/imu_balance/imu_balance.so")
ubx.load_module(ni, "std_blocks/servo_motor_driver/servo_motor_driver.so")
ubx.load_module(ni, "std_blocks/lfds_buffers/lfds_cyclic.so")
ubx.load_module(ni, "std_blocks/webif/webif.so")
ubx.load_module(ni, "std_blocks/logging/file_logger.so")
ubx.load_module(ni, "std_blocks/ptrig/ptrig.so")
ubx.load_module(ni, "std_blocks/random/random.so")

ubx.ffi_load_types(ni)

print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })

print("creating instance of 'imu_driver/imu_driver'")
imu_driver1=ubx.block_create(ni, "imu_driver/imu_driver", "imu_driver1", {sensor_config={sensor_base_addr=0x41600000, sensor_slave_addr=0xA6, sensor_data_addr=0x32}})

print("creating instance of 'random/random'")
random1=ubx.block_create(ni, "random/random", "random1", {min_max_config={min=0, max=1000}})

print("creating instance of 'servo_motor_driver/servo_motor_driver'")
servo_motor_driver1=ubx.block_create(ni, "servo_motor_driver/servo_motor_driver", "servo_motor_driver1", {axi_port_config={ZeroDegree=1442, usPwmPeriod=20000, usConst=100, nMotors=8, baseAddr=0x43C00000}})

print("creating instance of 'imu_balance/imu_balance'")
imu_balance1=ubx.block_create(ni, "imu_balance/imu_balance", "imu_balance1")

print("creating instance of 'lfds_buffers/cyclic'")
fifo1=ubx.block_create(ni, "lfds_buffers/cyclic", "fifo1", {buffer_len=4, type_name='float', data_len=3})
fifo2=ubx.block_create(ni, "lfds_buffers/cyclic", "fifo2", {buffer_len=8, type_name='unsigned int', data_len=8})
fifo3=ubx.block_create(ni, "lfds_buffers/cyclic", "fifo3", {buffer_len=8, type_name='unsigned int', data_len=8})
fifo4=ubx.block_create(ni, "lfds_buffers/cyclic", "fifo4", {buffer_len=8, type_name='unsigned int'})

print("creating instance of 'logging/file_logger'")

logger_conf=[[
{
   { blockname='imu_driver1', portname="result_arr", buff_len=2, data_len=3,}
}
]]

file_log1=ubx.block_create(ni, "logging/file_logger", "file_log1",
			   {filename='report.dat',
			    separator=',',
			    timestamp=1,
			    report_conf=logger_conf})

print("creating instance of 'std_triggers/ptrig'")
ptrig1=ubx.block_create(ni, "std_triggers/ptrig", "ptrig1",
			{
			   period = {sec=0, usec=5000 },
			   sched_policy="SCHED_OTHER", sched_priority=0,
			   trig_blocks={
					 { b=file_log1, num_steps=1, measure=0 },
			  		 { b=imu_balance1, num_steps=1, measure=0 },	
					 { b=imu_driver1, num_steps=1, measure=0 },
					 { b=servo_motor_driver1, num_steps=1, measure=0 },
					 { b=random1, num_steps=1, measure=0 },
			   } } )

--ubx.ni_stat(ni)

print("running webif init", ubx.block_init(webif1))
print("running ptrig1 init", ubx.block_init(ptrig1))
print("running imu_balance1 init", ubx.block_init(imu_balance1))
print("running fifo1 init", ubx.block_init(fifo1))
print("running fifo2 init", ubx.block_init(fifo2))
print("running fifo3 init", ubx.block_init(fifo3))
print("running fifo4 init", ubx.block_init(fifo4))
print("running file_log1 init", ubx.block_init(file_log1))
print("running imu_driver1 init", ubx.block_init(imu_driver1))
print("running servo_motor_driver1 init", ubx.block_init(servo_motor_driver1))

print("running webif start", ubx.block_start(webif1))

imu_result_port=ubx.port_get(imu_driver1, "result_arr")
servo_pwm_sig_out_port=ubx.port_get(servo_motor_driver1, "p_data_out")
servo_pwm_sig_in_port=ubx.port_get(servo_motor_driver1, "p_data_in")

mp_imu_in_port=ubx.port_get(imu_balance1, "in_port_imu")
mp_pwm_sig_in_port=ubx.port_get(imu_balance1 ,"in_port_pwm_sig");

--added by mildred/gorka
mp_random_sig_in_port=ubx.port_get(imu_balance1,"in_port_random");
random1_out_port=ubx.port_get(random1,"rnd");
--


mp_pwm_sig_out_port=ubx.port_get(imu_balance1 ,"out_port_pwm_sig");

ubx.port_connect_out(imu_result_port, fifo1)
ubx.port_connect_in(mp_imu_in_port,fifo1)

ubx.port_connect_out(servo_pwm_sig_out_port, fifo2)
ubx.port_connect_in(mp_pwm_sig_in_port ,fifo2)

ubx.port_connect_out(mp_pwm_sig_out_port, fifo3)
ubx.port_connect_in(servo_pwm_sig_in_port ,fifo3)

ubx.port_connect_out(random1_out_port, fifo4)
ubx.port_connect_in(mp_random_sig_in_port ,fifo4)

--ubx.block_start(file_log1)
--ubx.block_start(ptrig1)
ubx.block_start(fifo1)
ubx.block_start(fifo2)
ubx.block_start(fifo3)
ubx.block_start(fifo4)
ubx.block_start(imu_balance1)
ubx.block_start(imu_driver1)
ubx.block_start(servo_motor_driver1)

print("--- demo app launched, browse to http://localhost:8888 and start ptrig1 block to start up")
io.read()

print("stopping and cleaning up blocks --------------------------------------------------------")
print("running ptrig1 unload", ubx.block_unload(ni, "ptrig1"))
print("running webif1 unload", ubx.block_unload(ni, "webif1"))
print("running imu_balance unload", ubx.block_unload(ni, "imu_balance1"))
print("running file_log1 unload", ubx.block_unload(ni, "file_log1"))
print("running imu_driver1 unload", ubx.block_unload(ni, "imu_driver1"))
print("running fifo1 unload", ubx.block_unload(ni, "fifo1"))
print("running fifo2 unload", ubx.block_unload(ni, "fifo2"))
print("running fifo3 unload", ubx.block_unload(ni, "fifo3"))
print("running fifo4 unload", ubx.block_unload(ni, "fifo4"))
print("running servo_motor_driver1 unload", ubx.block_unload(ni, "servo_motor_driver1"))

ubx.node_cleanup(ni)
os.exit(1)
