void DMA(uint32_t ADD1,uint32_t ADD2,uint8_t frequency);
void DMA_Reset(void);

uint16_t AD0,AD1,AD2,AD3;

uint16_t Array2[4]={0,0,0,0};

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_10MHz,GPIO_Mode_AIN,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	ADC_InitTypeDef A1;
	A1.ADC_ContinuousConvMode=ENABLE;//������
	A1.ADC_DataAlign=ADC_DataAlign_Right;
	A1.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	A1.ADC_Mode=ADC_Mode_Independent;
	A1.ADC_NbrOfChannel=4;//4��ͨ��
	A1.ADC_ScanConvMode=ENABLE;//ɨ��
	ADC_Init(ADC1, &A1);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,3,ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,4,ADC_SampleTime_71Cycles5);
	
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_ResetCalibration(ADC1);//У׼
	while(ADC_GetResetCalibrationStatus(ADC1)==SET);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1)==SET);
	
	DMA((uint32_t)&ADC1->DR,(uint32_t)Array2,4);//��DR�д��䵽Array2
	
	ADC_DMACmd(ADC1,ENABLE);
	OLED_Init();
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);

	while(1)
	{
		DMA_Reset();
		OLED_ShowNum(1,1,Array2[0],4);
		OLED_ShowNum(2,1,Array2[1],4);
		OLED_ShowNum(3,1,Array2[2],4);
		OLED_ShowNum(4,1,Array2[3],4);
		Delayms(500);
	}

}

void DMA(uint32_t ADD2,uint32_t ADD1,uint8_t frequency)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//ʱ�ӳ�ʼ��	
	DMA_InitTypeDef D1;
	D1.DMA_BufferSize=frequency;//ת���������м�����ת����
	D1.DMA_DIR=DMA_DIR_PeripheralSRC;//ת��ģʽ
	D1.DMA_M2M=DMA_M2M_Disable;//Ӳ������
	D1.DMA_MemoryBaseAddr=ADD1;
	D1.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	D1.DMA_MemoryInc=DMA_MemoryInc_Enable;//�Ĵ�������
	D1.DMA_Mode=DMA_Mode_Normal;//ת��ģʽ
	D1.DMA_PeripheralBaseAddr=ADD2;
	D1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	D1.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//����Ĵ���������
	D1.DMA_Priority=DMA_Priority_VeryHigh;//���ȼ�
	DMA_Init(DMA1_Channel1, &D1);//ת��ͨ��
}
	
void DMA_Reset(void)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);//ʹ��
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);//�ȴ�ת�����
	DMA_Cmd(DMA1_Channel1,DISABLE);//ʧ��
	DMA_ClearFlag(DMA1_FLAG_TC1);//�ֶ������־λ
	DMA_SetCurrDataCounter(DMA1_Channel1,4);
}


