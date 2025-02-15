void DMA(uint32_t ADD1,uint32_t ADD2,uint8_t frequency)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//时钟初始化
	
	DMA_InitTypeDef D1;
	D1.DMA_BufferSize=frequency;//转换次数，有几个数转几次
	D1.DMA_DIR=DMA_DIR_PeripheralDST;
	D1.DMA_M2M=DMA_M2M_Enable;
	D1.DMA_MemoryBaseAddr=ADD1;
	D1.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	D1.DMA_MemoryInc=DMA_MemoryInc_Enable;
	D1.DMA_Mode=DMA_Mode_Normal;//转换模式
	D1.DMA_PeripheralBaseAddr=ADD2;
	D1.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	D1.DMA_PeripheralInc=DMA_PeripheralInc_Enable;
	D1.DMA_Priority=DMA_Priority_VeryHigh;//优先级
	DMA_Init(DMA1_Channel1, &D1);
}
 
 void DMA_Reset(uint8_t frequency)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);//使能
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);//等待转换完成
	DMA_Cmd(DMA1_Channel1,DISABLE);//失能
	DMA_ClearFlag(DMA1_FLAG_TC1);//手动清除标志位
}


