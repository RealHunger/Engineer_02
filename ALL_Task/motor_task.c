#include "motor_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "stdio.h"

// ====================== 新增【一键修改CAN总线】宏定义 - 核心需求 ✔️ ======================
#define CHASSIS_CAN_HANDLE       &hcan2   // 底盘大疆电机挂载的CAN总线，改这里一键切换
#define ARM_CAN_HANDLE           &hcan1   // 机械臂达妙电机挂载的CAN总线，改这里一键切换

void motor_task_func(void const * argument) {
    // 1. 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    struct motor_device* chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    for (int i = 0; i < 4; i++) {
        chassis[i]->init(chassis[i], 0x201 + i, CHASSIS_CAN_HANDLE, 6,  //替换为宏：CHASSIS_CAN_HANDLE
                         10.0, /* Kp */
                         0.0, /* Ki */
                         0.0, /* Kd */
                         10000.0, /* Max_Out: 最大电流输出 (max 16384) */
                         2000.0, /* I_Max: 积分限幅 */
                         0.5 /* Alpha: 微分项低通滤波系数 */
        );
    }

    struct motor_device* joint[7];
    for(int i=0; i<7; i++) {
        char name[25]; sprintf(name, "DM_JOINT%d", i+1);
        joint[i] = motor_get_device(name);
    }
    //joint[0]->init(joint[0], 0x01, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[1]->init(joint[1], 0x02, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[2]->init(joint[2], 0x03, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[3]->init(joint[3], 0x04, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[4]->init(joint[4], 0x05, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[5]->init(joint[5], 0x06, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    joint[6]->init(joint[6], 0x07, ARM_CAN_HANDLE,5,0.0f,0.0f,12.5f,3.0f,10.0f);
    // joint[0]->init(joint[0], 0x01, ARM_CAN_HANDLE,5,30.0f,5.0f,3.0f,45.0f,54.0f);
    // joint[1]->init(joint[1], 0x02, ARM_CAN_HANDLE,5,50.0f,5.0f,1.5f,25.0f,200.0f);
    // joint[2]->init(joint[2], 0x03, ARM_CAN_HANDLE,5,20.0f,5.0f,1.57f,10.0f,28.0f);
    // joint[3]->init(joint[3], 0x04, ARM_CAN_HANDLE,5,20.0f,2.5f,1.57f,10.0f,28.0f);
    // joint[4]->init(joint[4], 0x05, ARM_CAN_HANDLE,5,15.0f,2.0f,3.14f,3.0f,10.0f);
    // joint[5]->init(joint[5], 0x06, ARM_CAN_HANDLE,5,15.0f,2.0f,1.9f,3.0f,10.0f);
    // joint[6]->init(joint[6], 0x07, ARM_CAN_HANDLE,5,15.0f,2.0f,1.57f,3.0f,10.0f);


    struct motor_device *gripper = motor_get_device("DM_GRIPPER");
    gripper->init(gripper,0x08, ARM_CAN_HANDLE,5,15.0f,2.0f,12.5f,3.0f,10.0f);

    /************************ 达妙电机轮发数组定义 ************************/
    struct motor_device* dm_motors[8] = {joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], joint[6], gripper};
    static uint8_t dm_send_idx = 0;  // 达妙轮发索引 0-7，自动循环

    while (1)
    {
        // 1. 每毫秒执行所有电机PID计算，无此步骤电机无控制输出
        for(uint8_t i=0; i<4; i++) chassis[i]->update(chassis[i]);
        for(uint8_t i=0; i<8; i++) dm_motors[i]->update(dm_motors[i]);

        // 2. 大疆底盘电机指令
        DJI_Motor_Send_CAN_Group(CHASSIS_CAN_HANDLE);  //替换为宏：CHASSIS_CAN_HANDLE

        // 3. 8个达妙轮流发送,每毫秒仅发送1个，8ms轮完一遍，无总线压力
        dm_motors[dm_send_idx]->send_ctrl_cmd(dm_motors[dm_send_idx]);
        dm_send_idx = (dm_send_idx + 1) % 8;  // 索引自增取模，自动循环0-7

        osDelay(1);  // 精准1ms周期
    }
}