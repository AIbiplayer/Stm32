//配置GPIO
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_10MHz,GPIO_Mode_IN_FLOATING,GPIO_Pin_0|GPIO_Pin_1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//TIM2使能
	
	
	TIM_ICInitTypeDef T2;//捕获初始化
	T2.TIM_Channel=TIM_Channel_1|TIM_Channel_2;
	T2.TIM_ICFilter=0xF;
	T2.TIM_ICPolarity=TIM_ICPolarity_Rising;
	T2.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	T2.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &T2);
	
	TIM_TimeBaseInitTypeDef TI2;//配置TIM2
	TIM_TimeBaseStructInit(&TI2);
	TI2.TIM_ClockDivision=TIM_CKD_DIV1;
	TI2.TIM_CounterMode=TIM_CounterMode_Up;
	TI2.TIM_Period=65535;
	TI2.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM2, &TI2);
	
	//编码器接口
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM2, ENABLE);
	
	while(1)
	{	
		OLED_ShowString(1, 1, "count:");
		OLED_ShowSignedNum(2, 1, (int16_t)TIM_GetCounter(TIM2), 4);
	}
