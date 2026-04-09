#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include "stdarg.h"
#include "stdint.h"
#include "referee_protocol.h"
#include "referee.h"

#pragma pack(1) // 按1字节对齐

/* 此处的定义只与UI绘制有关 */
typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    uint8_t Delete_Operate; // 删除操作
    uint8_t Layer;
    uint16_t frametail;
} UI_delete_t;

typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    uint16_t frametail;
} UI_GraphReFresh_t;

typedef struct {
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    String_Data_t String_Data;
    uint16_t frametail;
} UI_CharReFresh_t; // 打印字符串数据

#pragma pack()

void DetermineRobotID(void);

referee_info_t *UIInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

void UIStart(void);

void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer);

void UILineDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void UIRectangleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                     uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void UICircleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                  uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

void UIOvalDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);

void UIArcDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x,
               uint32_t Start_y,
               uint32_t end_x, uint32_t end_y);

void UIFloatDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                 uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                 int32_t Graph_Float);

void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);

void UICharDraw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...);

void UIGraphRefresh(referee_id_t *_id, int cnt, ...);

void UICharRefresh(referee_id_t *_id, String_Data_t string_Data);

void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);

// 新增：重组后的UI绘制函数
void UI_Init_Midgroup(void);
void UI_Init_Ungroup(void);
void UI_Init_Upgroup(void);

/**
 * @brief 更新UPGROUP动态数据
 * @param track_head 履带头角度
 * @param track_back 履带尾角度
 * @param speed_target 目标速度
 * @param power_end_x 功率条终点X坐标
 * @param capenergy_end_x 电容能量条终点X坐标
 * @param chassis_mode 底盘模式字符串
 * @param gimbal_mode 云台模式字符串
 * @param friction_mode 摩擦轮模式字符串
 */
void UI_Update_Upgroup(uint32_t track_head, uint32_t track_back, int32_t speed_target,
                       uint32_t power_end_x, uint32_t capenergy_end_x,
                       const char *chassis_mode, const char *gimbal_mode, const char *friction_mode);

#endif
