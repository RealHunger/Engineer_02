#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"



void chassis_task_func(void const * argument) {
    /******************************************************************************************************************/
    /* 初始化 */
    // 调试串口
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取电机设备句柄
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();



        /**************************************************************************************************************/
        // 只修改：遥控器掉线检测 → VT13专属超时判定
        if (current_tick - robot_ctrl.rc->vt13.last_update_tick > 200) {
            // 遥控器掉线保护
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        } else {
            robot_ctrl.monitor.remote_online = 1;
            /**********************************************************************************************************/


        }
        osDelay(2);
    }
}