#include "arm_task.h"          // 云台任务头文件-本文件声明
#include "../Application/robot_global.h"  // 全局变量头文件-核心全局结构体、枚举定义
#include "../../Components/motor/motor.h" // 电机驱动头文件-电机设备句柄/接口函数
#include "math.h"                 // 数学库头文件-三角函数/绝对值/浮点运算
#include "stdlib.h"               // 标准库头文件-通用工具函数
#include "cmsis_os.h"             // RTOS系统头文件-系统滴答/延时/任务调度
#include "stdio.h"                // 标准输入输出-调试打印备用
#include "../../Bsp/uart/bsp_uart.h" // 串口驱动头文件-上位机/外设通信
#include "../../Bsp/usb_cdc/bsp_usb_cdc.h"
#include "../../Bsp/led/bsp_led.h"   // LED驱动头文件-状态指示灯控制【保留灯光 不删除】
#include "../../Components/remote/remote.h"
#include "../../Application/custom_ctrl_uart.h"


#include "../../Algorithm/arm_gravity_comp/robot_arm.h"

void arm_task_func(void const * argument) {
    /**************************************** 【硬件外设初始化区】 ****************************************/
    // 获取串口1 DMA句柄并初始化：波特率**921600**（协议要求）、8位数据位、无校验、1位停止位
    struct uart_device* Uart = uart_get_device("uart1_dma");
    // 关键修改：波特率从115200改为921600（RoboMaster图传链路强制要求）
    Uart->Init(Uart, 921600, 8, 'N', 1);

    // 移除USB相关初始化（不再使用USB CDC）
    struct usb_device *Usb = usb_get_device();
    Usb->Init(Usb);

    // 初始化自定义控制器UART发送状态（重置包序号）
    CustomCtrl_UART_Init();

    /**************************************** 【系统上电启动保护】 ****************************************/
    // 等待传感器就绪：陀螺仪/加速度计等传感器未就绪前，云台不动作，防止失控
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);  // 上电延时1s，等待所有外设/电机/传感器稳定，硬件防冲击

    struct motor_device* joint[7];

     for(int i=0; i<7; i++) {
         char name[25]; sprintf(name, "DM_JOINT%d", i+1);
         joint[i] = motor_get_device(name);
         joint[i]->send_disable_cmd(joint[i]);
         osDelay(50);
     }
    struct motor_device* gripper = motor_get_device("DM_GRIPPER");
    gripper->send_disable_cmd(gripper);
    osDelay(50);

    robot_ctrl.monitor.arm_ready = 1;

     float joint_pos[7];
     float gravity_torque[7];
     float pos_test[3];
    uint8_t gripper_open = 0;

    /**************************************** 【云台任务主循环 - 死循环永不退出】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();  // 获取当前系统滴答定时器值(ms)，用于所有计时逻辑

        for (int i = 0; i < 7; i++) {
            joint[i]->get_status(joint[i], "POS", &joint_pos[i]);
        }

        calculate_7axis_gravity_torque(joint_pos, gravity_torque, pos_test);

        for (int i = 0; i < 7; i++) {
            joint[i]->set_target(joint[i], 3, 0.0f, 0.0f, gravity_torque[i]);
        }
        gripper->set_target(gripper, 3, 0.0f, 0.0f, 0.0f);


        CustomCtrl_UART_Send_MotorData(Uart, joint_pos, gripper_open);


        //Uart->Print(Uart, "%f,%f,%f\r\n", pos_test[0], pos_test[1], pos_test[2]);

        Usb->Print(Usb, "%f\r\n", robot_ctrl.rc->custom.custom_ctrl.joint_angles_rad[0]);

        LED_BLUE_Toggle();

        osDelay(2);  // 云台任务调度周期 2ms，固定频率保证控制精度
    }
}