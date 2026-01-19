//
// Created by 14717 on 2025/8/27.
//

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "motor.h"
#include "math.h"
#include "cmsis_os.h"
#include "../../Bsp/LED/bsp_LED.h"

#define pi (fp32)M_PI

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**********************************************************************************************************************/
/* 达妙电机——MIT控制 【保留全套完整驱动，适配关节+末端夹爪】*/
struct DM_MIT_data {
    //控制量
    float _p_des;
    float _v_des;
    float _t_ff;

    //参数
    float _kp;
    float _kd;

    //反馈量
    int8_t ERR;  //电机错误状态反馈
    int16_t POS;  //当前电机位置反馈
    int16_t VEL;  //当前电机转速反馈
    int16_t T;  //当前电机转矩反馈
    int8_t TEMP_MOS;  //电机MOS温度反馈
    int8_t TEMP_Rotor;  //电机线圈温度反馈

    //控制幅值
    float P_MAX;
    float V_MAX;
    float T_MAX;
};

void DM_MIT_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL) return;
    if (motor->motor_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;
    memset(d, 0, sizeof(*d));
    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);

    if (hcan != NULL) {
        motor->motor_can_handle = hcan;
    }

    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);
        if (para_num >= 1) d->_kp = (float)va_arg(ap, double);
        if (para_num >= 2) d->_kd = (float)va_arg(ap, double);
        if (para_num >= 3) d->P_MAX = (float)va_arg(ap, double);
        if (para_num >= 4) d->V_MAX = (float)va_arg(ap, double);
        if (para_num >= 5) d->T_MAX = (float)va_arg(ap, double);
        va_end(ap);
    }

    // 发送使能帧
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef enable_tx_message;
    uint8_t enable_can_send_data[8];
    memset(&enable_tx_message, 0, sizeof(enable_tx_message));
    enable_tx_message.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    enable_tx_message.IDE = CAN_ID_STD;
    enable_tx_message.RTR = CAN_RTR_DATA;
    enable_tx_message.DLC = 0x08;
    for (int i = 0; i < 7; ++i) enable_can_send_data[i] = 0xFF;
    enable_can_send_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &enable_tx_message, enable_can_send_data, &send_mail_box);
}

void DM_get_measure(const struct motor_device *motor, const uint8_t *data)
{
    if (motor == NULL || motor->motor_data == NULL || data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    d->ERR = (int8_t)((data[0] >> 4) & 0x0F);
    d->POS = (int16_t)((uint16_t)data[1] << 8 | (uint16_t)data[2]);

    // 解析12位速度
    const uint16_t vel_raw = (uint16_t)data[3] << 8 | (uint16_t)data[4];
    int16_t vel12 = (int16_t)(vel_raw >> 4);
    if (vel12 & (1 << 11)) vel12 |= 0xF000;
    d->VEL = vel12;

    // 解析12位扭矩
    uint16_t t_raw = ((uint16_t)(data[4] & 0x0F) << 8) | (uint16_t)data[5];
    int16_t t12 = (int16_t)t_raw;
    if (t12 & (1 << 11)) t12 |= 0xF000;
    d->T = t12;

    d->TEMP_MOS = (int8_t)data[6];
    d->TEMP_Rotor = (int8_t)data[7];
}

void DM_update(struct motor_device *motor) {}

void DM_MIT_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    // 数据归一化打包
    uint16_t pdes = (uint16_t)((d->_p_des + d->P_MAX) / (d->P_MAX * 2.0f) * 65535.0f);
    uint16_t vdes = (int16_t)((d->_v_des + d->V_MAX) / (d->V_MAX * 2.0f) * 4095.0f);
    uint16_t kp = (int16_t)(d->_kp / 500.0f * 4095.0f);
    uint16_t kd = (int16_t)(d->_kd / 5.0f * 4095.0f);
    uint16_t t_ff = (int16_t)((d->_t_ff + d->T_MAX) / (d->T_MAX * 2.0f) * 4095.0f);
    // 限幅
    pdes = pdes > 0xFFFF ? 0xFFFF : (pdes < 0 ? 0 : pdes);
    vdes = vdes > 0x0FFF ? 0x0FFF : (vdes < 0 ? 0 : vdes);
    kp = kp > 0x0FFF ? 0x0FFF : (kp < 0 ? 0 : kp);
    kd = kd > 0x0FFF ? 0x0FFF : (kd < 0 ? 0 : kd);
    t_ff = t_ff > 0x0FFF ? 0x0FFF : (t_ff < 0 ? 0 : t_ff);

    // 组帧发送
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    tx_data[0] = (uint8_t)(pdes >> 8);
    tx_data[1] = (uint8_t)(pdes & 0xFF);
    tx_data[2] = (uint8_t)(vdes >> 4);
    tx_data[3] = (uint8_t)((vdes & 0x0F) << 4) | (uint8_t)((kp >> 8) & 0x0F);
    tx_data[4] = (uint8_t)(kp & 0xFF);
    tx_data[5] = (uint8_t)(kd >> 4);
    tx_data[6] = (uint8_t)((kd & 0x0F) << 4) | (uint8_t)((t_ff >> 8) & 0x0F);
    tx_data[7] = (uint8_t)(t_ff & 0xFF);

    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

void DM_MIT_send_disable_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;
    for (int i = 0; i < 7; ++i) tx_data[i] = 0xFF;
    tx_data[7] = 0xFD;
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

void DM_MIT_send_enable_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef enable_tx_message;
    uint8_t enable_can_send_data[8];
    memset(&enable_tx_message, 0, sizeof(enable_tx_message));
    enable_tx_message.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    enable_tx_message.IDE = CAN_ID_STD;
    enable_tx_message.RTR = CAN_RTR_DATA;
    enable_tx_message.DLC = 0x08;
    for (int i = 0; i < 7; ++i) enable_can_send_data[i] = 0xFF;
    enable_can_send_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &enable_tx_message, enable_can_send_data, &send_mail_box);
}

void DM_MIT_set_target(const struct motor_device *motor, const int para_num, ...) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;
    va_list ap;
    va_start(ap, para_num);
    if (para_num >= 1) d->_p_des = (float)va_arg(ap, double);
    if (para_num >= 2) d->_v_des = (float)va_arg(ap, double);
    if (para_num >= 3) d->_t_ff = (float)va_arg(ap, double);
    va_end(ap);
}

void DM_MIT_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    if (motor == NULL || motor->motor_data == NULL || which_status == NULL || status_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    if (strcmp(which_status, "ERR") == 0) *(int8_t *)status_data = d->ERR;
    else if (strcmp(which_status, "POS") == 0) {
        *(float *) status_data = (float) d->POS / 65535.0f * 2 * d->P_MAX;
        *(float *) status_data = *(float *)status_data > 0 ? *(float *)status_data - d->P_MAX : *(float *)status_data + d->P_MAX;
    }
    else if (strcmp(which_status, "VEL") == 0) {
        *(float *)status_data = (float)d->VEL / 4096.0f * 2 * d->V_MAX;
        *(float *)status_data = *(float *)status_data > 0 ? *(float *)status_data - d->V_MAX : *(float *)status_data + d->V_MAX;
    }
    else if (strcmp(which_status, "T") == 0) {
        *(float *)status_data = (float)d->T / 4096.0f * 2 * d->T_MAX;
        *(float *)status_data = *(float *)status_data > 0 ? *(float *)status_data - d->T_MAX : *(float *)status_data + d->T_MAX;
    }
    else if (strcmp(which_status, "TEMP_MOS") == 0) *(int8_t *)status_data = d->TEMP_MOS;
    else if (strcmp(which_status, "TEMP_Rotor") == 0) *(int8_t *)status_data = d->TEMP_Rotor;
    else if (strcmp(which_status, "P_MAX") == 0) *(float *)status_data = d->P_MAX;
    else if (strcmp(which_status, "V_MAX") == 0) *(float *)status_data = d->V_MAX;
    else if (strcmp(which_status, "T_MAX") == 0) *(float *)status_data = d->T_MAX;
    else if (strcmp(which_status, "Kp") == 0) *(float *)status_data = d->_kp;
    else if (strcmp(which_status, "Kd") == 0) *(float *)status_data = d->_kd;
    else if (strcmp(which_status, "p_des") == 0) *(float *)status_data = d->_p_des;
    else if (strcmp(which_status, "v_des") == 0) *(float *)status_data = d->_v_des;
    else if (strcmp(which_status, "tff") == 0) *(float *)status_data = d->_t_ff;
}

void DM_MIT_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    if (motor == NULL || motor->motor_data == NULL || which_para == NULL || para_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) {
        d->_kp = *(float *)para_data;
        d->_kp = d->_kp < 0 ? 0 : (d->_kp > 500 ? 500 : d->_kp);
    }
    else if (strcmp(which_para, "Kd") == 0) {
        d->_kd = *(float *)para_data;
        d->_kd = d->_kd < 0 ? 0 : (d->_kd > 5 ? 5 : d->_kd);
    }
    else if (strcmp(which_para, "P_MAX") == 0) d->P_MAX = *(float *)para_data;
    else if (strcmp(which_para, "V_MAX") == 0) d->V_MAX = *(float *)para_data;
    else if (strcmp(which_para, "T_MAX") == 0) d->T_MAX = *(float *)para_data;
}

/**********************************************************************************************************************/
/* M3508——速度控制 【保留全套完整驱动+底盘4个实例】*/
struct M3508_data {
    int16_t _v_des;          // 目标转速 (rpm)
    int16_t _current_output; // PID输出电流 (-16384 ~ 16384)
    float _kp;               // PID参数
    float _ki;
    float _kd;
    float _i_term;           // 积分项
    int16_t _last_error;     // 上一次误差
    float _last_d_out;       // 上一次微分输出
    float _d_filter_alpha;   // 微分滤波系数
    int16_t POS;             // 机械角度 (0~8191)
    int16_t VEL;             // 转速 (rpm)
    int16_t CURRENT;         // 实际电流
    int8_t TEMP;             // 温度
    int8_t ERR;              // 错误状态
    float _current_output_max;// 电流限幅
    float _i_output_max;     // 积分限幅
    uint8_t enable_flag;     // 使能标志
};

void M3508_VEL_PID_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    memset(d, 0, sizeof(struct M3508_data));

    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);
    if (hcan != NULL) motor->motor_can_handle = hcan;

    // 默认参数
    d->_current_output_max = 500.0f;
    d->_i_output_max = 500.0f;
    d->_d_filter_alpha = 1.0f;

    // 解析可变参数
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);
        if (para_num >= 1) d->_kp = (float)va_arg(ap, double);
        if (para_num >= 2) d->_ki = (float)va_arg(ap, double);
        if (para_num >= 3) d->_kd = (float)va_arg(ap, double);
        if (para_num >= 4) d->_current_output_max = (float)va_arg(ap, double);
        if (para_num >= 5) d->_i_output_max = (float)va_arg(ap, double);
        if (para_num >= 6) d->_d_filter_alpha = (float)va_arg(ap, double);
        va_end(ap);
    }
}

void M3508_get_measure(const struct motor_device *motor, const uint8_t *data)
{
    if (motor == NULL || motor->motor_data == NULL || data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    d->POS = (int16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    d->VEL = (int16_t)((uint16_t)data[2] << 8 | (uint16_t)data[3]);
    d->CURRENT = (int16_t)((uint16_t)data[4] << 8 | (uint16_t)data[5]);
    d->TEMP = (int8_t)data[6];
    d->ERR = (int8_t)data[7];
}

void M3508_enable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    ((struct M3508_data *)motor->motor_data)->enable_flag = 1;
}

void M3508_disable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    d->enable_flag = 0;
    d->_i_term = 0.0f;
    d->_current_output = 0;
}

void M3508_VEL_PID_update(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    if (d->enable_flag == 0) {
        d->_i_term = 0.0f;
        d->_current_output = 0;
        return;
    }

    // PID计算
    int16_t error = d->_v_des - d->VEL;
    float p_out = d->_kp * (float)error;

    // 积分项限幅
    d->_i_term += d->_ki * (float)error;
    d->_i_term = d->_i_term > d->_i_output_max ? d->_i_output_max : (d->_i_term < -d->_i_output_max ? -d->_i_output_max : d->_i_term);

    // 微分项滤波
    float current_d_raw = d->_kd * (float)(error - d->_last_error);
    float d_out = d->_d_filter_alpha * current_d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;

    d->_last_error = error;
    d->_last_d_out = d_out;

    // 总输出限幅
    float total_out = p_out + d->_i_term + d_out;
    total_out = total_out > d->_current_output_max ? d->_current_output_max : (total_out < -d->_current_output_max ? -d->_current_output_max : total_out);
    d->_current_output = (int16_t)total_out;
}

void M3508_VEL_PID_set_target(const struct motor_device *motor, const int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    va_list ap;
    va_start(ap, para_num);
    if (para_num >= 1) d->_v_des = (int16_t)va_arg(ap, double);
    va_end(ap);
}

void M3508_VEL_PID_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    if (motor == NULL || motor->motor_data == NULL || which_status == NULL || status_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    if (strcmp(which_status, "POS") == 0) *(int16_t *)status_data = d->POS;
    else if (strcmp(which_status, "VEL") == 0) *(int16_t *)status_data = d->VEL;
    else if (strcmp(which_status, "CURRENT") == 0) *(int16_t *)status_data = d->CURRENT;
    else if (strcmp(which_status, "TEMP") == 0) *(int8_t *)status_data = d->TEMP;
    else if (strcmp(which_status, "ERR") == 0) *(int8_t *)status_data = d->ERR;
    else if (strcmp(which_status, "v_des") == 0) *(int16_t *)status_data = d->_v_des;
    else if (strcmp(which_status, "Kp") == 0) *(float *)status_data = d->_kp;
    else if (strcmp(which_status, "Ki") == 0) *(float *)status_data = d->_ki;
    else if (strcmp(which_status, "Kd") == 0) *(float *)status_data = d->_kd;
    else if (strcmp(which_status, "current_max") == 0) *(float *)status_data = d->_current_output_max;
    else if (strcmp(which_status, "i_max") == 0) *(float *)status_data = d->_i_output_max;
    else if (strcmp(which_status, "alpha") == 0) *(float *)status_data = d->_d_filter_alpha;
}

void M3508_VEL_PID_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    if (motor == NULL || motor->motor_data == NULL || which_para == NULL || para_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) d->_kp = *(float *)para_data;
    else if (strcmp(which_para, "Ki") == 0) d->_ki = *(float *)para_data;
    else if (strcmp(which_para, "Kd") == 0) d->_kd = *(float *)para_data;
    else if (strcmp(which_para, "current_max") == 0) d->_current_output_max = *(float *)para_data;
    else if (strcmp(which_para, "i_max") == 0) {
        d->_i_output_max = *(float *)para_data;
        d->_i_term = d->_i_term > d->_i_output_max ? d->_i_output_max : (d->_i_term < -d->_i_output_max ? -d->_i_output_max : d->_i_term);
    }
    else if (strcmp(which_para, "alpha") == 0) {
        float a = *(float *)para_data;
        d->_d_filter_alpha = a < 0 ? 0 : (a > 1 ? 1 : a);
    }
}

/**********************************************************************************************************************/
/* 电机实例 - 7关节+1末端夹爪(达妙) + 4底盘M3508 无发射轮 */
// ===================== 达妙电机 - 7个关节 + 1个末端执行器(夹爪) ID:0x01~0x08 =====================
struct DM_MIT_data J4310_JOINT1_data = {0};
struct motor_device J4310_JOINT1 = {
    .motor_name = "DM_JOINT1",.motor_id = 0x01,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT1_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT2_data = {0};
struct motor_device J4310_JOINT2 = {
    .motor_name = "DM_JOINT2",.motor_id = 0x02,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT2_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT3_data = {0};
struct motor_device J4310_JOINT3 = {
    .motor_name = "DM_JOINT3",.motor_id = 0x03,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT3_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT4_data = {0};
struct motor_device J4310_JOINT4 = {
    .motor_name = "DM_JOINT4",.motor_id = 0x04,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT4_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT5_data = {0};
struct motor_device J4310_JOINT5 = {
    .motor_name = "DM_JOINT5",.motor_id = 0x05,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT5_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT6_data = {0};
struct motor_device J4310_JOINT6 = {
    .motor_name = "DM_JOINT6",.motor_id = 0x06,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT6_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

struct DM_MIT_data J4310_JOINT7_data = {0};
struct motor_device J4310_JOINT7 = {
    .motor_name = "DM_JOINT7",.motor_id = 0x07,.motor_can_handle = &hcan2,.motor_data = &J4310_JOINT7_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

// ✅ 第8个达妙 - 末端执行器/夹爪 (ID:0x08)
struct DM_MIT_data J4310_GRIPPER_data = {0};
struct motor_device J4310_GRIPPER = {
    .motor_name = "DM_GRIPPER",.motor_id = 0x08,.motor_can_handle = &hcan2,.motor_data = &J4310_GRIPPER_data,
    .init = DM_MIT_init,.get_measure = DM_get_measure,.update = DM_update,.send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,.send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,.get_status = DM_MIT_get_status,.set_para = DM_MIT_set_para
};

// ===================== M3508 4个 (底盘) ID:0x201~0x204 =====================
struct M3508_data M3508_CHASSIS_1_data = {0};
struct motor_device M3508_CHASSIS_1 = {
    .motor_name = "M3508_CHASSIS_1",.motor_id = 0x201,.motor_can_handle = &hcan1,.motor_data = &M3508_CHASSIS_1_data,
    .init = M3508_VEL_PID_init,.get_measure = M3508_get_measure,.update = M3508_VEL_PID_update,.send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,.send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,.get_status = M3508_VEL_PID_get_status,.set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_2_data = {0};
struct motor_device M3508_CHASSIS_2 = {
    .motor_name = "M3508_CHASSIS_2",.motor_id = 0x202,.motor_can_handle = &hcan1,.motor_data = &M3508_CHASSIS_2_data,
    .init = M3508_VEL_PID_init,.get_measure = M3508_get_measure,.update = M3508_VEL_PID_update,.send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,.send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,.get_status = M3508_VEL_PID_get_status,.set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_3_data = {0};
struct motor_device M3508_CHASSIS_3 = {
    .motor_name = "M3508_CHASSIS_3",.motor_id = 0x203,.motor_can_handle = &hcan1,.motor_data = &M3508_CHASSIS_3_data,
    .init = M3508_VEL_PID_init,.get_measure = M3508_get_measure,.update = M3508_VEL_PID_update,.send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,.send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,.get_status = M3508_VEL_PID_get_status,.set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_4_data = {0};
struct motor_device M3508_CHASSIS_4 = {
    .motor_name = "M3508_CHASSIS_4",.motor_id = 0x204,.motor_can_handle = &hcan1,.motor_data = &M3508_CHASSIS_4_data,
    .init = M3508_VEL_PID_init,.get_measure = M3508_get_measure,.update = M3508_VEL_PID_update,.send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,.send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,.get_status = M3508_VEL_PID_get_status,.set_para = M3508_VEL_PID_set_para
};

/**********************************************************************************************************************/
/* 电机列表 - 7关节+1夹爪(达妙) + 4底盘M3508 无冗余 */
struct motor_device *motor_list[] = {
    &J4310_JOINT1, &J4310_JOINT2, &J4310_JOINT3, &J4310_JOINT4,
    &J4310_JOINT5, &J4310_JOINT6, &J4310_JOINT7, &J4310_GRIPPER,
    &M3508_CHASSIS_1, &M3508_CHASSIS_2, &M3508_CHASSIS_3, &M3508_CHASSIS_4
};
/**********************************************************************************************************************/
#ifndef MOTOR_COUNT
#define MOTOR_COUNT (sizeof(motor_list) / sizeof(motor_list[0]))
#endif

/**
 * @brief 根据名称获取电机设备指针
 * @param name 注册时的实例化电机名称，可选值：
 *        DM_JOINT1、DM_JOINT2、DM_JOINT3、DM_JOINT4、
 *        DM_JOINT5、DM_JOINT6、DM_JOINT7、DM_GRIPPER、
 *        M3508_CHASSIS_1、M3508_CHASSIS_2、M3508_CHASSIS_3、M3508_CHASSIS_4
 * @return 指向电机实例的指针，未找到返回 NULL
 */
struct motor_device *motor_get_device(const char *name)
{
    if (name == NULL) return NULL;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (motor_list[i] != NULL && motor_list[i]->motor_name != NULL) {
            if (strcmp(motor_list[i]->motor_name, name) == 0) {
                return motor_list[i];
            }
        }
    }
    return NULL;
}
/**********************************************************************************************************************/
/* 功能：CAN1/CAN2 任意总线触发中断，都能自动解析达妙+M3508反馈，无需区分总线 */
/* 无任何 hcan1/hcan2 判断，纯按电机协议帧头解析，调试时随意切换电机挂载口 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 1. 获取CAN接收数据（通用，不管是CAN1还是CAN2）
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) return;

    // 2. 解析【达妙电机】反馈帧 - 协议固定帧头 StdId=0x00 (关节+夹爪)
    if(rx_header.StdId == 0x00)
    {
        uint8_t dm_motor_id = rx_data[0] & 0x0F; // 解析达妙电机ID 1-8
        struct motor_device *dm = NULL;
        if(dm_motor_id >=1 && dm_motor_id <=7)  // ID1-7 = 关节电机
        {
            char name_buf[32] = {0};
            sprintf(name_buf, "DM_JOINT%d", dm_motor_id);
            dm = motor_get_device(name_buf);
        }
        else if(dm_motor_id ==8) // ID8 = 末端夹爪
        {
            dm = motor_get_device("DM_GRIPPER");
        }
        if(dm != NULL) dm->get_measure(dm, rx_data); // 解析数据
        return;
    }

    // 3. 解析【M3508电机】反馈帧 - 协议固定帧头 0x201~0x208 (底盘电机)
    if(rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208)
    {
        // 遍历所有电机，匹配ID即可，不管挂载在哪个CAN口
        for(uint8_t i=0; i<MOTOR_COUNT; i++)
        {
            struct motor_device *m = motor_list[i];
            if(m != NULL && m->motor_id == rx_header.StdId)
            {
                m->get_measure(m, rx_data); // 解析数据
                break;
            }
        }
        return;
    }
}

void DJI_Motor_Send_CAN_Group(CAN_HandleTypeDef *hcan) {
    // 发送底盘M3508控制帧 (0x200)
    uint8_t tx_200[8] = {0};
    CAN_TxHeaderTypeDef header = {.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8, .StdId = 0x200};
    uint32_t mailbox;

    for (int i = 0; i < 4; i++) {
        char name[32];
        sprintf(name, "M3508_CHASSIS_%d", i + 1);
        struct motor_device *m = motor_get_device(name);
        if (m && m->motor_data) {
            struct M3508_data *d = (struct M3508_data *)m->motor_data;
            int16_t out = (d->enable_flag) ? d->_current_output : 0;
            tx_200[i*2]   = (uint8_t)(out >> 8);
            tx_200[i*2+1] = (uint8_t)(out & 0xFF);
        }
    }
    HAL_CAN_AddTxMessage(hcan, &header, tx_200, &mailbox);
}