int main(void)
{
	OLED_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_9);//TX
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_10);//RX

	USART_InitTypeDef UInit1;
	UInit1.USART_BaudRate=9600;
	UInit1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	UInit1.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//发送模式
	UInit1.USART_Parity=USART_Parity_No;//不校验
	UInit1.USART_StopBits=USART_StopBits_1;//1停止位
	UInit1.USART_WordLength=USART_WordLength_8b;//8位长度
	USART_Init(USART1,&UInit1);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//USART1中断，当收到数据时启动中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NInit;
	NInit.NVIC_IRQChannel=USART1_IRQn;//USART1中断
	NInit.NVIC_IRQChannelCmd=ENABLE;
	NInit.NVIC_IRQChannelPreemptionPriority=1;
	NInit.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NInit);
	
	USART_Cmd(USART1,ENABLE);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);

	while(1)
	{
	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
	OLED_ShowNum(1,1,(uint32_t)USART_ReceiveData(USART1),4);
	Delayms(500);
	USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
}

