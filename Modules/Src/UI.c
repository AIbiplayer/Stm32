/**
 * @file referee_UI.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-1-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "referee.h"

static referee_info_t *referee_recv_info; // 裁判系统数据指针,由Referee_Init()返回
static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态

uint8_t UI_Seq;

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3]; // 电容能量条
static String_Data_t UI_State_sta[6]; // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6]; // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DetermineRobotID() {
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data) {
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.
        chassis_power_mx) {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.
                chassis_power_mx;
    }
}


/**
 * @brief  UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 * @param  referee_info_t *referee_recv_info 裁判系统数据指针
 * @param  Referee_Interactive_info_t *_Interactive_data UI绘制需要的机器人状态
 * @retval none
 * @attention 该函数会根据flag判断哪些模式切换了，刷新对应的UI，刷新完成后将flag清零
 */
void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data) {
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1) {
        switch (_Interactive_data->chassis_mode) {
            case CHASSIS_ZERO_FORCE:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
                break;
            case CHASSIS_ROTATE:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
                // 此处注意字数对齐问题，字数相同才能覆盖掉
                break;
            case CHASSIS_NO_FOLLOW:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "nofollow ");
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1) {
        switch (_Interactive_data->gimbal_mode) {
            case GIMBAL_ZERO_FORCE: {
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
                break;
            }
            case GIMBAL_FREE_MODE: {
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
                break;
            }
            case GIMBAL_GYRO_MODE: {
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
                break;
            }
            case GIMBAL_VISION:
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // shoot
    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1) {
        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650,
                   _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1) {
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600,
                   _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1) {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550,
                   _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1) {
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230,
                    _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160,
                   (uint32_t) 750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
}

referee_info_t *UIInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data) {
    referee_recv_info = Referee_Init(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data; // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 0;
    return referee_recv_info;
}

void UIStart(void) {
    DetermineRobotID(); // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210,
               shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340,
               shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110,
               shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110,
               shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110,
               shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2],
                   UI_shoot_line[3], UI_shoot_line[4]);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 底盘功率显示，静态
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

// 包序号
/********************************************删除操作*************************************
**参数：_id 对应的id结构体
		Del_Operate  对应头文件删除操作
		Del_Layer    要删除的层 取值0-9
*****************************************************************************************/
void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer) {
    static UI_delete_t UI_delete_data;
    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del; // 计算交互数据长度

    UI_delete_data.FrameHeader.SOF = REFEREE_SOF;
    UI_delete_data.FrameHeader.DataLength = temp_datalength;
    UI_delete_data.FrameHeader.Seq = UI_Seq;
    UI_delete_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *) &UI_delete_data, LEN_CRC8, 0xFF);

    UI_delete_data.CmdID = ID_student_interactive;

    UI_delete_data.datahead.data_cmd_id = UI_Data_ID_Del;
    UI_delete_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_delete_data.datahead.sender_ID = _id->Robot_ID;

    UI_delete_data.Delete_Operate = Del_Operate; // 删除操作
    UI_delete_data.Layer = Del_Layer;

    UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t *) &UI_delete_data,
                                                   LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);
    /* 填入0xFFFF,关于crc校验 */

    RefereeSend((uint8_t *) &UI_delete_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

    UI_Seq++; // 包序号+1
}

/************************************************绘制直线*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y  起点xy坐标
		End_x、End_y   终点xy坐标
**********************************************************************************************************/

void UILineDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) // 填充至‘0’为止
    {
        graph->graphic_name[2 - i] = graphname[i]; // 按内存地址增大方向填充，所以会有i与2-i
    }

    graph->operate_tpye = Graph_Operate;
    graph->graphic_tpye = UI_Graph_Line;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    起点xy坐标
		End_x、End_y        对角顶点xy坐标
**********************************************************************************************************/
void UIRectangleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                     uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Rectangle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    圆心xy坐标
		Graph_Radius  圆形半径
**********************************************************************************************************/

void UICircleDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                  uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Circle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Radius;
    graph->end_x = 0;
    graph->end_y = 0;
}

/************************************************绘制椭圆*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_y    圆心xy坐标
		End_x、End_y        xy半轴长度
**********************************************************************************************************/
void UIOvalDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Ellipse;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;

    graph->start_angle = 0;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

/************************************************绘制圆弧*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_StartAngle,Graph_EndAngle    起始终止角度
		Graph_Width    图形线宽
		Start_y,Start_y    圆心xy坐标
		x_Length,y_Length   xy半轴长度
**********************************************************************************************************/

void UIArcDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x,
               uint32_t Start_y,
               uint32_t end_x, uint32_t end_y) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }

    graph->graphic_tpye = UI_Graph_Arc;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = Graph_StartAngle;
    graph->end_angle = Graph_EndAngle;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = 0;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

/************************************************绘制浮点型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Digit    小数位数
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标
		radius=a&0x3FF;   a为浮点数乘以1000后的32位整型数
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/

void UIFloatDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                 uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
                 int32_t Graph_Float) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }
    graph->graphic_tpye = UI_Graph_Float;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->start_angle = Graph_Size;
    graph->end_angle = Graph_Digit;

    graph->radius = Graph_Float & 0x3FF;
    graph->end_x = (Graph_Float >> 10) & 0x7FF;
    graph->end_y = (Graph_Float >> 21) & 0x7FF;
}

/************************************************绘制整型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标
		radius=a&0x3FF;   a为32位整型数
		end_x=(a>>10)&0x7FF;
		end_y=(a>>21)&0x7FF;
**********************************************************************************************************/
void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
               uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->graphic_name[2 - i] = graphname[i];
    }
    graph->graphic_tpye = UI_Graph_Int;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;

    graph->start_angle = Graph_Size;
    graph->end_angle = 0;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Integer & 0x3FF;
    graph->end_x = (Graph_Integer >> 10) & 0x7FF;
    graph->end_y = (Graph_Integer >> 21) & 0x7FF;
}

/************************************************绘制字符型数据*************************************************
**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		graphname[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Size     字号
		Graph_Width    图形线宽
		Start_x、Start_y    开始坐标

**参数：*graph Graph_Data类型变量指针，用于存放图形数据
		fmt需要显示的字符串
		此函数的实现和具体使用类似于printf函数
**********************************************************************************************************/
void UICharDraw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...) {
    int i;
    for (i = 0; i < 3 && graphname[i] != '\0'; i++) {
        graph->Graph_Control.graphic_name[2 - i] = graphname[i];
    }

    graph->Graph_Control.graphic_tpye = UI_Graph_Char;
    graph->Graph_Control.operate_tpye = Graph_Operate;
    graph->Graph_Control.layer = Graph_Layer;
    graph->Graph_Control.color = Graph_Color;

    graph->Graph_Control.width = Graph_Width;
    graph->Graph_Control.start_x = Start_x;
    graph->Graph_Control.start_y = Start_y;
    graph->Graph_Control.start_angle = Graph_Size;
    graph->Graph_Control.radius = 0;
    graph->Graph_Control.end_x = 0;
    graph->Graph_Control.end_y = 0;

    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *) graph->show_Data, fmt, ap); // 使用参数列表进行格式化并输出到字符串
    va_end(ap);
    graph->Graph_Control.end_angle = strlen((const char *) graph->show_Data);
}

/* UI推送函数（使更改生效）
   参数： cnt   图形个数
			...   图形变量参数
   Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
 */
void UIGraphRefresh(referee_id_t *_id, int cnt, ...) {
    UI_GraphReFresh_t UI_GraphReFresh_data;
    Graph_Data_t graphData;

    uint8_t temp_datalength = LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * cnt +
                              LEN_TAIL; // 计算交互数据长度

    static uint8_t buffer[512]; // 交互数据缓存

    va_list ap; // 创建一个 va_list 类型变量
    va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表

    UI_GraphReFresh_data.FrameHeader.SOF = REFEREE_SOF;
    UI_GraphReFresh_data.FrameHeader.DataLength = Interactive_Data_LEN_Head + cnt * UI_Operate_LEN_PerDraw;
    UI_GraphReFresh_data.FrameHeader.Seq = UI_Seq;
    UI_GraphReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *) &UI_GraphReFresh_data, LEN_CRC8, 0xFF);

    UI_GraphReFresh_data.CmdID = ID_student_interactive;

    switch (cnt) {
        case 1:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw1;
            break;
        case 2:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw2;
            break;
        case 5:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw5;
            break;
        case 7:
            UI_GraphReFresh_data.datahead.data_cmd_id = UI_Data_ID_Draw7;
            break;
    }

    UI_GraphReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_GraphReFresh_data.datahead.sender_ID = _id->Robot_ID;
    memcpy(buffer, (uint8_t *) &UI_GraphReFresh_data, LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head);
    // 将帧头、命令码、交互数据帧头三部分复制到缓存中

    for (uint8_t i = 0; i < cnt; i++) // 发送交互数据的数据帧，并计算CRC16校验值
    {
        graphData = va_arg(ap, Graph_Data_t); // 访问参数列表中的每个项,第二个参数是你要返回的参数的类型,在取值时需要将其强制转化为指定类型的变量
        memcpy(buffer + (LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * i),
               (uint8_t *) &graphData, UI_Operate_LEN_PerDraw);
    }
    Append_CRC16_Check_Sum(buffer, temp_datalength);
    RefereeSend(buffer, temp_datalength);

    va_end(ap); // 结束可变参数的获取
}

/************************************************UI推送字符（使更改生效）*********************************/
void UICharRefresh(referee_id_t *_id, String_Data_t string_Data) {
    static UI_CharReFresh_t UI_CharReFresh_data;

    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar; // 计算交互数据长度

    UI_CharReFresh_data.FrameHeader.SOF = REFEREE_SOF;
    UI_CharReFresh_data.FrameHeader.DataLength = temp_datalength;
    UI_CharReFresh_data.FrameHeader.Seq = UI_Seq;
    UI_CharReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *) &UI_CharReFresh_data, LEN_CRC8, 0xFF);

    UI_CharReFresh_data.CmdID = ID_student_interactive;

    UI_CharReFresh_data.datahead.data_cmd_id = UI_Data_ID_DrawChar;

    UI_CharReFresh_data.datahead.receiver_ID = _id->Cilent_ID;
    UI_CharReFresh_data.datahead.sender_ID = _id->Robot_ID;

    UI_CharReFresh_data.String_Data = string_Data;

    UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t *) &UI_CharReFresh_data,
                                                        LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

    RefereeSend((uint8_t *) &UI_CharReFresh_data, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

    UI_Seq++; // 包序号+1
}
