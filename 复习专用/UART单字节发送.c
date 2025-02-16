void Send_String(char *a);
void HPrintf(char *a,...);//可变参数封装，用来写入字符串，最多99字符
uint32_t Change(uint32_t a,uint8_t common);//次方转换
void Send_Num(int32_t a,uint8_t Length);//发送数字

int main(void)
{
	OLED_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_AF_PP,GPIO_Pin_9);//TX
	GPIOX_Init(GPIOA,RCC_APB2Periph_GPIOA,GPIO_Speed_50MHz,GPIO_Mode_IPU,GPIO_Pin_10);//RX

	USART_InitTypeDef UInit1;
	UInit1.USART_BaudRate=9600;
	UInit1.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控
	UInit1.USART_Mode=USART_Mode_Tx;//发送模式
	UInit1.USART_Parity=USART_Parity_No;//不校验
	UInit1.USART_StopBits=USART_StopBits_1;//1停止位
	UInit1.USART_WordLength=USART_WordLength_8b;//8位长度
	USART_Init(USART1,&UInit1);
	
	/*USART_ClockInitTypeDef UC1;
	UC1.USART_Clock=USART_Clock_Enable;//开启时钟
	UC1.USART_CPHA=
	UC1.USART_CPOL=
	UC1.USART_LastBit=
	USART_ClockInit(USART1, &UC1);*/
	
	USART_Cmd(USART1,ENABLE);
	HPrintf("申飞麟");
	
//uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
	
	while(1)
	{
		//OLED_ShowHexNum(1,1,USART_ReceiveData(USART1),4);
		//USART_SendData(USART1, 0x66);
	}
}

void Send_String(char* a)//发送字符串
{
	for(uint8_t i=0;a[i]!=0;i++)
	{
		USART_SendData(USART1,a[i]);
		Delayms(5);//发送后等待
	}
}

uint32_t Change(uint32_t a,uint8_t common)//次方转换
{
	uint32_t i=1;
	if(!common)
	{
		return 1;
	}
	while(common)
	{
		i*=a;
		common--;
	}
	return i;
}

void Send_Num(int32_t a,uint8_t Length)//发送数字
{
	for(uint8_t i=0;i<Length;i++)
	{
		USART_SendData(USART1,a/Change(10,Length-i-1)%10+48);
		Delayms(5);//发送后等待
	}
}

int fputc(int a,FILE*f)//改造printf函数，注意魔术棒里要UseMicroLIB
{
	USART_SendData(USART1,a);
	Delayms(5);//发送后等待	
	return a;
}

void HPrintf(char *a,...)//可变参数封装，用来写入字符串，最多99字符 
{
//记得#include <stdarg.h>
 
	char String[100];
	va_list arg;
	va_start(arg,a);
	vsprintf(String,a,arg);
	va_end(arg);
	Send_String(a);
}
