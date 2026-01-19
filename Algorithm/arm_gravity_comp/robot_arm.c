#include <stdio.h>
#include <math.h>
#include <string.h> // 新增：用于memset（可选，初始化用）

// 宏定义：7轴机械臂的关节数量
#define JOINT_NUM_7AXIS 7
// 圆周率常量（若系统math.h已定义M_PI，可注释此宏，避免重复定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 2π常量（角度周期）
#define M_2PI (2.0f * M_PI)

float angle_wrap_pi(float angle) {
    // 步骤1：计算角度对2π的模，得到余数（处理周期）
    float wrapped = fmod(angle, M_2PI);

    // 步骤2：调整余数到[-π, π]范围
    if (wrapped > M_PI) {
        wrapped -= M_2PI;
    } else if (wrapped < -M_PI) {
        wrapped += M_2PI;
    }

    // 步骤3：处理浮点数精度问题（比如因计算误差导致的微小超出范围）
    if (fabs(wrapped - M_PI) < 1e-6) {
        wrapped = M_PI;
    } else if (fabs(wrapped + M_PI) < 1e-6) {
        wrapped = -M_PI;
    }

    return wrapped;
}

// 三维向量结构体：表示三维空间中的点或向量（x,y,z分量）
typedef struct {
    float x, y, z;
} Vector3f;

// 3x3旋转矩阵结构体：表示三维空间中的旋转变换（正交矩阵）
typedef struct {
    float m[3][3];
} Matrix3f;

// 4x4齐次变换矩阵结构体：表示三维空间的旋转+平移变换
typedef struct {
    float m[4][4];
} Matrix4f;

// DH参数结构体（标准DH参数：alpha, a, d, theta）
// 说明：标准DH参数用于描述相邻两个关节（连杆）之间的位姿关系
typedef struct {
    float d;      // 连杆偏距：沿z轴的平移量（单位：米）
    float a;      // 连杆长度：沿x轴的平移量（单位：米）
    float alpha;  // 连杆扭转角：绕x轴的旋转角（单位：弧度）
    float theta;  // 关节转角：绕z轴的旋转角（单位：弧度）
} DHParam;

// 连杆状态结构体：存储单个连杆的位姿、质量、质心等信息
// 说明：包含相对位姿（相对于前一连杆）和世界系位姿（相对于基坐标系）
typedef struct {
    /* 相对位姿（i连杆相对于i+1连杆，或i+1相对于i） */
    Matrix4f T;          // 4x4齐次变换矩阵：描述i+1连杆相对于i连杆的位姿
    Vector3f pos;        // 位置向量：i+1连杆相对于i连杆的位置
    Matrix3f rot;        // 旋转矩阵：i+1连杆相对于i连杆的旋转
    Vector3f z_axis;     // 旋转轴：第i关节的旋转轴（z轴，相对i连杆）

    /* 自身属性（连杆的物理属性） */
    float mass;          // 连杆质量（单位：kg）
    Vector3f m_pos;      // 质心坐标：质心相对于i连杆的位置
    Vector3f G;

    /* 世界属性（绝对位姿） */
    Matrix4f w_T;        // 4x4齐次变换矩阵：描述i连杆相对于世界系的位姿
    Vector3f w_pos;      // 连杆末端坐标：i连杆原点在世界系中的位置（基坐标系）
    Matrix3f w_rot;      // 旋转矩阵：i连杆相对于世界系的旋转
    Vector3f w_m_pos;    // 质心坐标：质心在世界系中的位置（计算：w_m_pos = w_pos + w_rot * m_pos（旋转矩阵乘质心向量））
    Vector3f w_z_axis;   // 旋转轴：第i关节的旋转轴在世界系中的方向（计算：w_z_axis = w_rot * z_axis（旋转矩阵乘z轴向量））
} LinkState;

// ====================== 向量操作工具函数 ======================
/**
 * @brief 向量缩放：将向量v乘以缩放因子scale，结果存入res
 * @param v 输入向量（源向量）
 * @param scale 缩放因子（标量）
 * @param res 输出向量（缩放后的结果）
 */
void vector3f_scale(const Vector3f* v, float scale, Vector3f* res) {
    res->x = v->x * scale;
    res->y = v->y * scale;
    res->z = v->z * scale;
}

/**
 * @brief 向量加法：v1 + v2，结果存入res
 * @param v1 输入向量1
 * @param v2 输入向量2
 * @param res 输出向量（求和结果）
 */
void vector3f_add(const Vector3f* v1, const Vector3f* v2, Vector3f* res) {
    res->x = v1->x + v2->x;
    res->y = v1->y + v2->y;
    res->z = v1->z + v2->z;
}

/**
 * @brief 向量减法：v1 - v2，结果存入res
 * @param v1 输入向量1（被减数）
 * @param v2 输入向量2（减数）
 * @param res 输出向量（相减结果）
 */
void vector3f_sub(const Vector3f* v1, const Vector3f* v2, Vector3f* res) {
    res->x = v1->x - v2->x;
    res->y = v1->y - v2->y;
    res->z = v1->z - v2->z;
}

/**
 * @brief 向量叉乘：v1 × v2，结果存入res（叉乘结果是垂直于v1和v2的向量）
 * @param v1 输入向量1
 * @param v2 输入向量2
 * @param res 输出向量（叉乘结果）
 */
void vector3f_cross(const Vector3f* v1, const Vector3f* v2, Vector3f* res) {
    // 叉乘公式：v1×v2 = (y1z2 - z1y2, z1x2 - x1z2, x1y2 - y1x2)
    res->x = v1->y * v2->z - v1->z * v2->y;
    res->y = v1->z * v2->x - v1->x * v2->z;
    res->z = v1->x * v2->y - v1->y * v2->x;
}

/**
 * @brief 向量点乘：v1 · v2，返回标量结果（点乘结果表示两向量的夹角余弦乘以模长）
 * @param v1 输入向量1
 * @param v2 输入向量2
 * @return 点乘结果（标量）
 */
float vector3f_dot(const Vector3f* v1, const Vector3f* v2) {
    // 点乘公式：v1·v2 = x1x2 + y1y2 + z1z2
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

// ====================== 矩阵操作工具函数 ======================
/**
 * @brief 4x4矩阵置为单位矩阵：对角线元素为1，其余为0
 * @param m 输入输出矩阵：需要置为单位矩阵的4x4矩阵
 */
void matrix4f_identity(Matrix4f* m) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            m->m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 3x3矩阵乘以三维向量：m × v，结果存入res（实现旋转向量的操作）
 * @param m 输入3x3矩阵（旋转矩阵）
 * @param v 输入三维向量
 * @param res 输出三维向量（矩阵乘向量的结果）
 */
void matrix3f_mul_vector3f(const Matrix3f* m, const Vector3f* v, Vector3f* res) {
    // 矩阵乘向量公式：res[i] = sum(m[i][j] * v[j])，j从0到2
    res->x = m->m[0][0] * v->x + m->m[0][1] * v->y + m->m[0][2] * v->z;
    res->y = m->m[1][0] * v->x + m->m[1][1] * v->y + m->m[1][2] * v->z;
    res->z = m->m[2][0] * v->x + m->m[2][1] * v->y + m->m[2][2] * v->z;
}

/**
 * @brief 4x4矩阵乘法：m1 × m2，结果存入res（矩阵乘法遵循行乘列规则）
 * @param m1 输入第一个4x4矩阵（左矩阵）
 * @param m2 输入第二个4x4矩阵（右矩阵）
 * @param res 输出4x4矩阵（矩阵乘法结果）
 */
void matrix4f_multiply(const Matrix4f* m1, const Matrix4f* m2, Matrix4f* res) {
    Matrix4f temp; // 临时变量存储结果，避免覆盖输入
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp.m[i][j] = 0.0f;
            // 矩阵乘法核心：第i行第j列 = 左矩阵第i行 · 右矩阵第j列
            for (int k = 0; k < 4; k++) {
                temp.m[i][j] += m1->m[i][k] * m2->m[k][j];
            }
        }
    }
    *res = temp; // 将临时结果赋值给输出矩阵
}

/**
 * @brief 从4x4齐次矩阵中提取位置向量（前3行第4列）
 * @param m 输入4x4齐次矩阵
 * @param pos 输出三维向量（位置信息）
 */
void matrix4f_extract_position(const Matrix4f* m, Vector3f* pos) {
    pos->x = m->m[0][3];
    pos->y = m->m[1][3];
    pos->z = m->m[2][3];
}

/**
 * @brief 从4x4齐次矩阵中提取z轴向量（前3行第3列），并归一化为单位向量
 * @param m 输入4x4齐次矩阵
 * @param z_axis 输出三维向量（z轴方向，单位向量）
 */
void matrix4f_extract_z_axis(const Matrix4f* m, Vector3f* z_axis) {
    // 提取旋转矩阵的z轴（齐次矩阵前3行第3列）
    z_axis->x = m->m[0][2];
    z_axis->y = m->m[1][2];
    z_axis->z = m->m[2][2];
    // 归一化：确保向量为单位向量（鲁棒性处理，避免数值误差）
    float norm = sqrt(z_axis->x * z_axis->x + z_axis->y * z_axis->y + z_axis->z * z_axis->z);
    if (norm > 1e-6) { // 避免除以0（浮点数精度问题）
        z_axis->x /= norm;
        z_axis->y /= norm;
        z_axis->z /= norm;
    }
}

/**
 * @brief 从4x4齐次矩阵中提取3x3旋转矩阵（前3行前3列）
 * @param m 输入4x4齐次矩阵
 * @param rot 输出3x3旋转矩阵
 */
void matrix4f_extract_rot(const Matrix4f* m, Matrix3f* rot) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot->m[i][j] = m->m[i][j];
        }
    }
}

// 生成绕x轴旋转指定角度（弧度）的4x4齐次变换矩阵
/**
 * @brief 生成绕x轴旋转θ弧度的4x4齐次变换矩阵（纯旋转，无平移）
 * @param theta 旋转角度（弧度）
 * @param m 输出的4x4齐次变换矩阵
 */
void matrix4f_rotate_x(float theta, Matrix4f* m) {
    // 先初始化单位矩阵
    matrix4f_identity(m);
    // 预计算三角函数值
    float c = cos(theta);
    float s = sin(theta);
    // 填充旋转部分（3x3旋转矩阵）
    m->m[1][1] = c;
    m->m[1][2] = -s;
    m->m[2][1] = s;
    m->m[2][2] = c;
    // 平移部分为0（单位矩阵已初始化）
}

// 生成绕y轴旋转指定角度（弧度）的4x4齐次变换矩阵（纯旋转，无平移）
/**
 * @brief 生成绕y轴旋转θ弧度的4x4齐次变换矩阵（右手坐标系，纯旋转）
 * @param theta 旋转角度（弧度）
 * @param m 输出的4x4齐次变换矩阵
 */
void matrix4f_rotate_y(float theta, Matrix4f* m) {
    // 先初始化单位矩阵（保证平移部分为0，最后一行为[0,0,0,1]）
    matrix4f_identity(m);
    // 预计算三角函数值
    float c = cos(theta);
    float s = sin(theta);
    // 填充旋转部分（3x3旋转矩阵，绕y轴的核心元素）
    m->m[0][0] = c;
    m->m[0][2] = s;
    m->m[2][0] = -s;
    m->m[2][2] = c;
    // 平移部分为0（单位矩阵已初始化）
}

// 生成绕z轴旋转指定角度（弧度）的4x4齐次变换矩阵（纯旋转，无平移）
/**
 * @brief 生成绕z轴旋转θ弧度的4x4齐次变换矩阵（右手坐标系，纯旋转）
 * @param theta 旋转角度（弧度）
 * @param m 输出的4x4齐次变换矩阵
 */
void matrix4f_rotate_z(float theta, Matrix4f* m) {
    // 先初始化单位矩阵（保证平移部分为0，最后一行为[0,0,0,1]）
    matrix4f_identity(m);
    // 预计算三角函数值（减少重复计算，提升效率）
    float c = cos(theta);
    float s = sin(theta);
    // 填充旋转部分（3x3旋转矩阵，绕z轴的核心元素）
    m->m[0][0] = c;
    m->m[0][1] = -s;
    m->m[1][0] = s;
    m->m[1][1] = c;
    // 平移部分为0（单位矩阵已初始化）
}

/**
 * @brief 根据标准DH参数生成4x4齐次变换矩阵（i连杆到i+1连杆的变换）
 * @param dh 输入DH参数结构体
 * @param m 输出4x4齐次变换矩阵
 */
void dh_param_to_matrix(const DHParam* dh, Matrix4f* m) {
    // 预计算三角函数值，减少重复计算
    float c_theta = cos(dh->theta); // cos(theta)
    float s_theta = sin(dh->theta); // sin(theta)
    float c_alpha = cos(dh->alpha); // cos(alpha)
    float s_alpha = sin(dh->alpha); // sin(alpha)

    // 标准DH齐次变换矩阵公式（核心）
    m->m[0][0] = c_theta;
    m->m[0][1] = -s_theta * c_alpha;
    m->m[0][2] = s_theta * s_alpha;
    m->m[0][3] = dh->a * c_theta;

    m->m[1][0] = s_theta;
    m->m[1][1] = c_theta * c_alpha;
    m->m[1][2] = -c_theta * s_alpha;
    m->m[1][3] = dh->a * s_theta;

    m->m[2][0] = 0.0f;
    m->m[2][1] = s_alpha;
    m->m[2][2] = c_alpha;
    m->m[2][3] = dh->d;

    m->m[3][0] = 0.0f;
    m->m[3][1] = 0.0f;
    m->m[3][2] = 0.0f;
    m->m[3][3] = 1.0f;
}

// ====================== 前向运动学函数 ======================
int forward_kinematics(LinkState* Link_list, int Link_num, DHParam* dh_params) {
    // 参数合法性校验（增强：覆盖空指针、无效长度）
    if (Link_list == NULL || Link_num <= 0 || dh_params == NULL) {
        return -1;
    }

    // 更新各Link本地信息（移除冗余的identity初始化）
    for (int i = 0; i < Link_num; i++) {
        // 直接根据DH参数生成本地变换矩阵（无需先初始化单位矩阵）
        dh_param_to_matrix(&dh_params[i], &Link_list[i].T);
        // 提取本地位置、z轴、旋转矩阵
        matrix4f_extract_position(&Link_list[i].T, &Link_list[i].pos);
        matrix4f_extract_z_axis(&Link_list[i].T, &Link_list[i].z_axis);
        matrix4f_extract_rot(&Link_list[i].T, &Link_list[i].rot);
    }
    Matrix4f temp;

    // 更新各Link世界信息
    for (int i = 0; i < Link_num; i++) {
        if (i == 0) {
            // 第一个连杆：世界变换矩阵 = 本地变换矩阵（简化逻辑，移除冗余乘法）
            Link_list[i].w_T = Link_list[i].T;
        }
        // else if (i == 2) {
        //     matrix4f_multiply(&Link_list[i - 1].w_T, &Link_list[i].T, &Link_list[i].w_T);
        //     matrix4f_rotate_y(M_PI / 2, &temp);
        //     matrix4f_multiply(&Link_list[i].w_T, &temp, &Link_list[i].w_T);
        // }
        else {
            // 后续连杆：世界变换矩阵 = 上一个连杆的世界矩阵 * 当前连杆的本地矩阵
            matrix4f_multiply(&Link_list[i - 1].w_T, &Link_list[i].T, &Link_list[i].w_T);
        }

        // 提取世界坐标系下的位置、z轴、旋转矩阵
        matrix4f_extract_position(&Link_list[i].w_T, &Link_list[i].w_pos);
        matrix4f_extract_z_axis(&Link_list[i].w_T, &Link_list[i].w_z_axis);
        matrix4f_extract_rot(&Link_list[i].w_T, &Link_list[i].w_rot);

        // 计算质心的世界位置：旋转矩阵*本地质心位置 + 连杆世界位置
        // 注意：m_pos需要在调用函数前初始化，否则结果错误
        matrix3f_mul_vector3f(&Link_list[i].w_rot, &Link_list[i].m_pos, &Link_list[i].w_m_pos);
        vector3f_add(&Link_list[i].w_m_pos, &Link_list[i].w_pos, &Link_list[i].w_m_pos);
    }

    // 关键修复：函数正常执行后返回0（表示成功）
    return 0;
}

// ====================== 你的重力补偿扭矩计算函数（核心修改） ======================
int calculate_7axis_gravity_torque(const float* theta_input, float* torque_output, float* pos_test ) {
    // 1. 参数合法性校验
    if (theta_input == NULL || torque_output == NULL) {
        printf("Error: 输入/输出数组不能为空！\n");
        return -1;
    }



    // 初始化7轴DH参数
    DHParam dh_params[JOINT_NUM_7AXIS] = {
        {0.0f, 0.0f, 0.0f,0.0f},   //link1
        {0.0f, 0.0f, M_PI/2,0.0f},   //link2
        {0.0f, 0.0f, -M_PI/2,0.0f},   //link3
        {0.3f,0.0f,-M_PI/2,0.0f},   //link4
        {0.0f, 0.1f, M_PI/2,0.0f},   //link5
        {0.2f, 0.0f,-M_PI/2,0.0f},   //link6
        {0.0f, 0.0f,M_PI/2,0.0f}    //link7
    };

    int test_num = 6;
    LinkState LinkList[JOINT_NUM_7AXIS] = {
        {.mass = 0.3f, .m_pos = {0.0f, 0.0f, 0.0f}},
        {.mass = 0.3f, .m_pos = {0.0f, 0.0f, 0.0f}},
        {.mass = 1.6f, .m_pos = {0.0f, 0.0f, 0.2f}},
        {.mass = 0.5f, .m_pos = {0.0f, 0.0f, 0.0f}},
        {.mass = 0.8f, .m_pos = {0.0f, 0.0f, 0.07f}},
        {.mass = 0.45f, .m_pos = {0.0f, 0.0f, -0.05f}},
        {.mass = 0.35f, .m_pos = {0.0f, 0.0f, 0.1f}},
    };

    for (int i = 0; i < JOINT_NUM_7AXIS - 1; i++) {
        dh_params[i + 1].theta = angle_wrap_pi(theta_input[i]);
    }

    forward_kinematics(LinkList, JOINT_NUM_7AXIS, dh_params);

    Vector3f g = {0.0f, 0.0f, -9.8f};

    //计算各link重力
    for (int i = 0; i < JOINT_NUM_7AXIS; i++) {
        vector3f_scale(&g, LinkList[i].mass, &LinkList[i].G);
    }

    // 计算各linkTorque
    for (int i = 0; i < JOINT_NUM_7AXIS; i++) {
        Vector3f torque = {0.0f, 0.0f, 0.0f};
        Vector3f torque_all = {0.0f, 0.0f, 0.0f};
        for (int j = i; j < JOINT_NUM_7AXIS; j++) {
            Vector3f r;
            vector3f_sub(&LinkList[j].w_m_pos, &LinkList[i].w_pos, &r);
            vector3f_cross(&r, &LinkList[j].G, &torque);
            vector3f_add(&torque_all, &torque, &torque_all);
        }
        torque_output[i] = -vector3f_dot(&torque_all, &LinkList[i].w_z_axis);
    }

    //torque_output[4] = -torque_output[4];

    //torque_output[5] = -torque_output[5];

    pos_test[0] = LinkList[6].w_pos.x;
    pos_test[1] = LinkList[6].w_pos.y;
    pos_test[2] = LinkList[6].w_pos.z;

    return 0; // 计算成功
}