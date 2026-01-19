#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

/**
 * @brief 7轴机械臂重力补偿扭矩计算（对外唯一接口）
 * @param arm_pose 输入：7个关节的角度数组（单位：弧度），数组长度必须为7
 * @param comp_torque 输出：7个关节的重力补偿扭矩数组（单位：N·m），数组长度必须为7
 * @return 0：计算成功，-1：计算失败（输入/输出数组为空或参数非法）
 */
int calculate_7axis_gravity_torque(const float* arm_pose, float* comp_torque, float* pos_test);

#endif // ROBOT_ARM_H