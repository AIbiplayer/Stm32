OLED_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//ADC和PA0使能
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//RCC中ADC分频设置，6以上
	
	GPIO_InitTypeDef GA;//PA0初始化
	GPIO_StructInit(&GA);
	GA.GPIO_Pin=GPIO_Pin_0;
	GA.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GA);
	
	ADC_InitTypeDef AD;//AD初始化
	AD.ADC_ContinuousConvMode=DISABLE;
	AD.ADC_DataAlign=ADC_DataAlign_Right;
	AD.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	AD.ADC_Mode= ADC_Mode_Independent;
	AD.ADC_NbrOfChannel=1;
	AD.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1, &AD);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_7Cycles5);//ADC规则组通道设置
	
	ADC_Cmd(ADC1, ENABLE);//启动ADC
	
	while(1)
	{	
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);//软件触发启动
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==SET);
		Delayms(100);
		OLED_ShowString(1,1,"V:");
		OLED_ShowNum(2,1,ADC_GetConversionValue(ADC1),6);//获取转换结果
	}
