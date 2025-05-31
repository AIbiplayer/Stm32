/**
 * @brief 灯珠特效，使用PD12引脚
 * @date 2025/5/26
 */

#ifndef GRB_H
#define GRB_H

#include "main.h"

extern uint8_t LED_Array[61][24];

typedef struct GRB ///< 绿红蓝，范围0~255
{
    uint8_t Color_G;
    uint8_t Color_R;
    uint8_t Color_B;
} GRB;

typedef struct HSV ///< HSV模型
{
    uint16_t H; ///< 色相
    float S;    ///< 饱和度
    float V;    ///< 明度
} HSV;

void GRB_Send(void);
void GRB_Close(void);
void GRB_Breath(HSV *Color);
void GRB_Medicine_Successful(void);
void HSV_To_GRB(const HSV *Hsv, GRB *Grb);
void GRB_Write(uint8_t LED_List, GRB Color_GRB);
void GRB_Distance_Breath(HSV *Color, float Distance);

#endif
