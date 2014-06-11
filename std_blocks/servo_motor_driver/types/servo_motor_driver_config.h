struct servo_motor_driver_config {
	unsigned int baseAddr;
	int nMotors;
	int usPwmPeriod;
	int usConst;	
	int ZeroDegree;
	char runMode;
};

