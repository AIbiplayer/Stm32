/**
 * @file Infrare.h
 * @brief 红外接收驱动代码及按键扫描头文件
 * @date 2025/9/30
 */

#ifndef __INFRARE_H__
#define __INFRARE_H__

#include "main.h"
#include "stdbool.h"

void Inf_Server(void);
void Infrare_Stop(void);
void Infrare_Start(void);

#endif // __INFRARE_H__
