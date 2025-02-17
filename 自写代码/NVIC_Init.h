/**
*@file NVIC_Init.h
*@author Éê·É÷ë
*@brief NVIC³õÊ¼»¯
*@date 2025/2/10 
*/

#ifndef NVIC_INIT_H
#define NVIC_INIT_H
 
void NV_Init(uint8_t NVIC_IRQChannelX,uint8_t NVIC_IRQChannelPreemptionPriorityX,uint8_t NVIC_IRQChannelSubPriorityX,uint32_t NVIC_PriorityGroup);      
/*void EXTI15_10_IRQHandler(void); //ä¸­æ–­å‡½æ•°ï¼Œé»˜è®¤11å¼•è„š*/

#endif
