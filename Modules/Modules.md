# 模块部分文档
更新于2025/11/28
## 各模块概述
### ax_delay
[name]:https://www.xtark.cn/ "塔克创新"
* 实现延时功能，直接调用系统SysTick，支持微秒和毫秒，是塔克创新的例程文件
* *注意要初始化后使用！*
### ax_ps2
* 手柄按键读取，也是例程文件，每*30ms*运行一次
* *注意接线顺序！*
### ax_sys
* 定义一些引脚，没什么好说的
### Bluetooth
* 塔克创新的蓝牙模块，串口通信，波特率固定115200
* 由于APP上有遥控、拨杆、重力模式，有些键位是相似的，我尝试在.h文件中使用结构体+联合体的方式来定义按键，目前可以使用
```
  typedef struct
    {
    Bluetooth_Mode_e Mode; // 当前模式

    int8_t X_L; // X方向数据（前进为正方向）
    int8_t Y_L; // Y方向数据（左边为正方向）
    Dif_Data_u Dif_Data; // 不同模式特有的数据
    Rocker_Handle_Data_s Rocker_Handle_Data; // 摇杆/手柄共有数据
    } Bluetooth_Data_s;
```
>重力模式下的YAW轴只进行了解算，没有实际应用
### Debug_Tool
* 从学长开源框架中得到的灵感，其实就是串口输出遥控器数据，方便调试
* 这里加入了CCMRAM的使用，目前没有完全了解，只对变量使用
```
#define CCMRAM_CODE __attribute__((section(".ccmram_code")))
#define CCMRAM_DATA __attribute__((section(".ccmram_data")))
```
### Mpu6050
* 网上搬的陀螺仪模块，硬件HAL库I2C通信，Yaw零漂比较大
* 加入李强强同学的简单解算，可以获得三轴数据
### OLED
[name]:https://www.bilibili.com/video/BV1th411z7sn/?p=10&share_source=copy_web&vd_source=fc3a87058d66c464ed25bae9cb0302ec "江协科技"
* 也是网上搬的显示屏模块，软件I2C通信，基于江协科技的代码修改


