// 建议重命名为：custom_ctrl_uart.h
#ifndef INFANTRY_01_CUSTOM_CTRL_UART_H
#define INFANTRY_01_CUSTOM_CTRL_UART_H

#include "stdint.h"
#include "cmsis_os.h"
// 移除USB CDC头文件，替换为UART头文件
#include "../../Bsp/uart/bsp_uart.h"

/******************************************************************************************
 *                                   协议宏定义（严格遵循RoboMaster 2026协议，完全保留）
 ******************************************************************************************/
#define FRAME_HEADER_LEN         5u         // 帧头长度：SOF(1B)+data_length(2B)+seq(1B)+CRC8(1B)
#define CMD_ID_LEN               2u         // 命令码长度（2字节）
#define FRAME_TAIL_LEN           2u         // 帧尾CRC16校验长度
#define SOF_VALUE                0xA5u      // 帧起始标识（协议表1-3强制要求）
#define CUSTOM_CMD_ID            0x0302     // 自定义控制器→机器人命令码（图传链路，协议表1-4）
#define DATA_LEN_MOTOR_ANGLE     30u        // data段长度（协议要求0x0302对应30字节）
#define TOTAL_FRAME_LEN          (FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LEN_MOTOR_ANGLE + FRAME_TAIL_LEN) // 总帧长39字节
#define JOINT_COUNT              7u         // 关节数量（7个）
#define GRIPPER_STATE_LEN        1u         // 夹爪状态长度（1字节bool）

/******************************************************************************************
 *                                   函数声明（修改为UART发送）
 ******************************************************************************************/
/**
 * @brief  UART1发送7个关节弧度 + 1个夹爪开合状态（适配30字节data段）
 * @param  uart_dev: UART设备句柄（通过uart_get_device("uart1_dma")获取）
 * @param  joint_angles_rad: 7个关节弧度数组（float[7]，原始弧度值，无转换）
 * @param  gripper_open: 夹爪开合状态（true=打开，false=闭合，存储为1字节：0x00=闭合，0x01=打开）
 * @return 0=发送成功，-1=参数错误，-2=发送失败
 */
int CustomCtrl_UART_Send_MotorData(struct uart_device *uart_dev, float joint_angles_rad[JOINT_COUNT], uint8_t gripper_open);

/**
 * @brief  初始化自定义控制器UART发送状态
 * @note   重置包序号seq，确保每次上电/重启后通信起始状态一致（协议可靠性要求）
 */
void CustomCtrl_UART_Init(void);

#endif // INFANTRY_01_CUSTOM_CTRL_UART_H