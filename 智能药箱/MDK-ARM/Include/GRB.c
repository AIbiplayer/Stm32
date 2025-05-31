/**
 * @brief 灯珠特效，使用PD12引脚
 * @date 2025/5/26
 */

/*****************
 * 写入的LED数组类型必须为uint8_t
 *****************/

#include "main.h"
#include "string.h"
#include "DHT11.h"
#include "HC_SR04.h"
#include "math.h"
#include "GRB.h"

extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim4;
extern uint32_t ADC_Light_Data;

#define BREATH_STEPS 30   ///< 呼吸灯变化步数，越大越细腻，但更耗内存
#define LED_MAX_NUMBER 60 ///< 最多60个灯珠
#define DOWN 48           ///< 低电平PWM
#define UP 80             ///< 高电平脉冲

uint8_t AWDG_Status = 0;                         ///< 模拟看门狗状态位，未设置夜灯模式为0
uint8_t Light_Status = 0;                        ///< 夜间开关灯状态，开灯为1
uint8_t Day_Night_Status = 0;                    ///< 白天和夜晚状态，夜晚为1
uint8_t LED_Array[LED_MAX_NUMBER + 1][24] = {0}; ///< 灯珠数组，多一位用于复位
uint32_t ADC_Light_Data = 0;                     ///< 光敏值，范围为0~4095

/**
 * @brief 向一个灯珠写入数据
 * @param LED_List 灯珠序号，0~59
 * @param Color_GRB 色彩
 */
void GRB_Write(uint8_t LED_List, GRB Color_GRB)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (Color_GRB.Color_G & 0x80)
        {
            LED_Array[LED_List][i] = UP;
        }
        else
        {
            LED_Array[LED_List][i] = DOWN;
        }
        Color_GRB.Color_G <<= 1;

        if (Color_GRB.Color_R & 0x80)
        {
            LED_Array[LED_List][i + 8] = UP;
        }
        else
        {
            LED_Array[LED_List][i + 8] = DOWN;
        }
        Color_GRB.Color_R <<= 1;

        if (Color_GRB.Color_B & 0x80)
        {
            LED_Array[LED_List][i + 16] = UP;
        }
        else
        {
            LED_Array[LED_List][i + 16] = DOWN;
        }
        Color_GRB.Color_B <<= 1;
    }
}

/**
 * @brief 发送灯珠数据，修改一次发送一次
 */
void GRB_Send(void)
{
    HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_SetCounter(&htim4, 0);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)LED_Array, sizeof(LED_Array) / sizeof(uint8_t));
    DHT11_Delay_us(1);
}

/**
 * @brief 关闭所有灯
 */
void GRB_Close(void)
{
    for (uint8_t i = 0; i < LED_MAX_NUMBER; i++)
    {
        for (uint8_t k = 0; k < 24; k++)
        {
            LED_Array[i][k] = DOWN;
        }
    }
    GRB_Send();
}

/**
 * @brief 将HSV颜色空间转换为GRB颜色空间
 * @param Hsv 输入HSV颜色结构体指针
 * @param Grb 输出GRB颜色结构体指针
 */
void HSV_To_GRB(const HSV *Hsv, GRB *Grb)
{
    float H = Hsv->H % 360; // 确保色相在0-359范围内
    float S = (Hsv->S > 1.0f) ? 1.0f : ((Hsv->S < 0.0f) ? 0.0f : Hsv->S);
    float V = (Hsv->V > 1.0f) ? 1.0f : ((Hsv->V < 0.0f) ? 0.0f : Hsv->V);

    float C = V * S;
    float X = C * (1.0f - fabsf(fmodf(H / 60.0f, 2.0f) - 1.0f));
    float M = V - C;

    float R1, G1, B1;

    if (H < 60)
    {
        R1 = C;
        G1 = X;
        B1 = 0;
    }
    else if (H < 120)
    {
        R1 = X;
        G1 = C;
        B1 = 0;
    }
    else if (H < 180)
    {
        R1 = 0;
        G1 = C;
        B1 = X;
    }
    else if (H < 240)
    {
        R1 = 0;
        G1 = X;
        B1 = C;
    }
    else if (H < 300)
    {
        R1 = X;
        G1 = 0;
        B1 = C;
    }
    else
    {
        R1 = C;
        G1 = 0;
        B1 = X;
    }

    Grb->Color_R = (uint8_t)((R1 + M) * 255);
    Grb->Color_G = (uint8_t)((G1 + M) * 255);
    Grb->Color_B = (uint8_t)((B1 + M) * 255);
}

/**
 * @brief 闪烁呼吸灯效果，用于提醒服药
 * @param Color HSV颜色
 */
void GRB_Breath(HSV *Color)
{
    GRB GRB_Color = {0};

    for (uint16_t k = 0; k < BREATH_STEPS; k++) // 渐亮（二次方缓动）
    {
        float t = (float)k / (BREATH_STEPS - 1);
        Color->V = t * t; // 二次方缓入
        HSV_To_GRB(Color, &GRB_Color);

        for (uint8_t i = 0; i < 60; i++)
        {
            GRB_Write(i, GRB_Color);
        }
        GRB_Send();
        HAL_Delay(50);
    }

    for (uint16_t k = 0; k < BREATH_STEPS; k++) // 渐暗（二次方缓动）
    {

        float t = (float)k / (BREATH_STEPS - 1);
        Color->V = 1.0f - t * t; // 二次方缓出
        HSV_To_GRB(Color, &GRB_Color);

        for (uint8_t i = 0; i < 60; i++)
        {
            GRB_Write(i, GRB_Color);
        }
        GRB_Send();
        HAL_Delay(50);
    }
}

/**
 * @brief 服药成功提示
 */
void GRB_Medicine_Successful(void)
{
    GRB GRB_Color = {0};
    GRB_Color.Color_G = 255;

    for (uint8_t i = 0; i < 60; i++)
    {
        GRB_Write(i, GRB_Color);
    }
    GRB_Send();
    HAL_Delay(3000);
    GRB_Close();
}

/**
 * @brief 亮度随距离变化的呼吸灯
 * @param Color 颜色
 * @param Distance 距离
 */
void GRB_Distance_Breath(HSV *Color, float Distance)
{
    if (Distance > 0.1f && Distance < 450.0f)
    {
        GRB GRB_Color = {0};

        Color->V = 0.005f / (Distance * 0.001f);
        HSV_To_GRB(Color, &GRB_Color);

        for (uint8_t i = 0; i < 60; i++)
        {
            GRB_Write(i, GRB_Color);
        }
        GRB_Send();
    }
    else if (Distance > 500.0f && Distance < 1000.0f)
    {
        GRB GRB_Color = {0};

        Color->V = 0.01f;
        HSV_To_GRB(Color, &GRB_Color);

        for (uint8_t i = 0; i < 60; i++)
        {
            GRB_Write(i, GRB_Color);
        }
        GRB_Send();
    }
}

/**
 * @brief 按键上沿触发夜间开关灯(PG0)
 */
void EXTI0_IRQHandler(void)
{
    Light_Status = !Light_Status;
    if (Light_Status == 1) // 开灯红灯亮，关灯红灯灭
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief 光敏传感器，看门狗值在3200~3600间，生成时注意查看
 */
void ADC4_IRQHandler(void)
{
    if (Light_Status == 1 && AWDG_Status == 0)
    {
        if (ADC_Light_Data > 3600) // 夜晚判定，蓝灯亮
        {
            Day_Night_Status = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        }
        else if (ADC_Light_Data < 3200) // 白天判定，蓝灯灭
        {
            Day_Night_Status = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        AWDG_Status = 1;
        __HAL_ADC_DISABLE_IT(&hadc4, ADC_IT_AWD);
    }
    HAL_ADC_IRQHandler(&hadc4);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (ADC_Light_Data >= 3200 && ADC_Light_Data <= 3600)
    {
        AWDG_Status = 0;
        __HAL_ADC_ENABLE_IT(&hadc4, ADC_IT_AWD);
    }
}