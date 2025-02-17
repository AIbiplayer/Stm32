OLED_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//ADC��PA0ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//RCC��ADC��Ƶ���ã�6����
	
	GPIO_InitTypeDef GA;//PA0��ʼ��
	GPIO_StructInit(&GA);
	GA.GPIO_Pin=GPIO_Pin_0;
	GA.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GA);
	
	ADC_InitTypeDef AD;//AD��ʼ��
	AD.ADC_ContinuousConvMode=DISABLE;
	AD.ADC_DataAlign=ADC_DataAlign_Right;
	AD.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	AD.ADC_Mode= ADC_Mode_Independent;
	AD.ADC_NbrOfChannel=1;
	AD.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1, &AD);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_7Cycles5);//ADC������ͨ������
	
	ADC_Cmd(ADC1, ENABLE);//����ADC
	
	while(1)
	{	
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);//�����������
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==SET);
		Delayms(100);
		OLED_ShowString(1,1,"V:");
		OLED_ShowNum(2,1,ADC_GetConversionValue(ADC1),6);//��ȡת�����
	}
