int main(void)
{
	GPIOX_Init(GPIOB, RCC_APB2Periph_GPIOB, GPIO_Speed_10MHz, GPIO_Mode_AF_OD, GPIO_Pin_8 | GPIO_Pin_9); // OLED输入
	OLED_Init();
	
	MPU6050_Init();
	
	SensorData Sensor;
	
	while (1)
	{
		MPU6050_GetData(&Sensor);
		OLED_ShowSignedNum(1, 1, Sensor.ACCEL_X, 4);
		OLED_ShowSignedNum(2, 1, Sensor.ACCEL_Y, 4);
		OLED_ShowSignedNum(3, 1, Sensor.ACCEL_Z, 4);
		OLED_ShowSignedNum(1, 8, Sensor.GYRO_X, 4);
		OLED_ShowSignedNum(2, 8, Sensor.GYRO_Y, 4);
		OLED_ShowSignedNum(3, 8, Sensor.GYRO_Z, 4);
		Delayms(200);
	}
}
