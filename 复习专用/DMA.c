void DMA(uint32_t ADD1,uint32_t ADD2,uint8_t frequency)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//ʱ�ӳ�ʼ��
	
	DMA_InitTypeDef D1;
	D1.DMA_BufferSize=frequency;//ת���������м�����ת����
	D1.DMA_DIR=DMA_DIR_PeripheralDST;
	D1.DMA_M2M=DMA_M2M_Enable;
	D1.DMA_MemoryBaseAddr=ADD1;
	D1.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	D1.DMA_MemoryInc=DMA_MemoryInc_Enable;
	D1.DMA_Mode=DMA_Mode_Normal;//ת��ģʽ
	D1.DMA_PeripheralBaseAddr=ADD2;
	D1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	D1.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
	D1.DMA_Priority=DMA_Priority_VeryHigh;//���ȼ�
	DMA_Init(DMA1_Channel1, &D1);
}
 
 void DMA_Reset(uint8_t frequency)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);//ʹ��
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);//�ȴ�ת�����
	DMA_Cmd(DMA1_Channel1,DISABLE);//ʧ��
	DMA_ClearFlag(DMA1_FLAG_TC1);//�ֶ������־λ
}


