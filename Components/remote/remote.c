#include "remote.h"
#include "cmsis_os.h"
#include "usart.h"
#include "string.h"
#include "../../Bsp/LED/bsp_LED.h"

/******************************************************************************************
 *                                   外设句柄声明 (原文件保留)
 ******************************************************************************************/
extern UART_HandleTypeDef huart3;  // DT7 -> USART3
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart6;  // VT13/自定义控制器 -> USART6（图传链路）
extern DMA_HandleTypeDef hdma_usart6_rx;

/******************************************************************************************
 *                                   双缓冲区定义 (独立分配，互不干扰)
 ******************************************************************************************/
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM_DT7];  // DT7 DMA双缓冲区
static uint8_t rc_rx_buf[2][RC_RX_BUF_SIZE_VT13];    // VT13 DMA双缓冲区
static uint8_t custom_rx_buf[2][CUSTOM_CTRL_FRAME_TOTAL_LEN]; // 自定义控制器DMA双缓冲区
static RC_ctrl_t remote_ctrl;                        // 合并后的总遥控器数据，全局唯一

/******************************************************************************************
 *                                   VT13 官方 CRC16 校验表 (原VT13保留)
 ******************************************************************************************/
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/******************************************************************************************
 *                                   VT13 CRC16校验 (原VT13保留)
 ******************************************************************************************/
static uint16_t RC_CRC16_Check(uint8_t *p_msg, uint16_t len)
{
    uint16_t crc16 = 0xFFFF;
    while (len--)
    {
        crc16 = (crc16 >> 8) ^ crc16_tab[(crc16 ^ *p_msg++) & 0x00ff];
    }
    return crc16;
}

/******************************************************************************************
 *                                   静态辅助函数：帧头CRC8计算（适配自定义控制器）
 ******************************************************************************************/
static uint8_t FrameHeader_CRC8_Calc(uint8_t *frame_header)
{
    // 官方提供的CRC8_TAB表（CRC8 MAXIM，协议附录一）
    const unsigned char CRC8_TAB[256] =
    {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
    };

    uint8_t crc8 = 0xFF;  // CRC8初始值（官方协议要求0xFF）
    if (frame_header == NULL)
    {
        return 0xFF;
    }

    // 仅校验帧头前4字节（SOF + data_length + seq），符合官方协议逻辑
    for (int i = 0; i < 4; i++)
    {
        crc8 = CRC8_TAB[(crc8 ^ frame_header[i]) & 0xFF];
    }
    return crc8;
}

/******************************************************************************************
 *                                   自定义控制器 帧头校验（适配协议表1-2）
 ******************************************************************************************/
static uint8_t CustomCtrl_FrameHeader_Check(uint8_t *frame) {
    // 1. 校验SOF（第0字节必须为0xA5）
    if (frame[0] != CUSTOM_CTRL_SOF) return 0;

    // 2. 校验帧头CRC8（第4字节为前4字节的CRC8结果）
    uint8_t calc_crc8 = FrameHeader_CRC8_Calc(frame);
    if (frame[4] != calc_crc8) return 0;

    // 3. 校验数据段长度（协议要求数据段30字节，帧头第1-2字节为data_length）
    uint16_t data_len = (frame[2] << 8) | frame[1];
    if (data_len != CUSTOM_CTRL_DATA_LEN) return 0;

    return 1;
}

/******************************************************************************************
 *                                   自定义控制器 数据解析（适配发送格式）
 ******************************************************************************************/
static void CustomCtrl_Data_Parse(volatile const uint8_t *frame, RC_custom_t *custom) {
    if (frame == NULL || custom == NULL) return;

    // 1. 跳过帧头（5字节）和命令码（2字节），定位到数据段起始位置
    const uint8_t *data_start = frame + 5 + 2;

    // 2. 解析7个关节弧度值（float类型，共7*4=28字节）
    memcpy(custom->custom_ctrl.joint_angles_rad, data_start,
           CUSTOM_CTRL_JOINT_COUNT * sizeof(float));

    // 3. 解析夹爪状态（第28字节，1字节）
    custom->custom_ctrl.gripper_open = data_start[28]; // 0=闭合，1=打开（与发送端一致）

    // 4. 更新时间戳（复用系统滴答定时器）
    custom->last_update_tick = osKernelSysTick();
}

/******************************************************************************************
 *                                   DT7 数据解析 (原DT7保留，适配新结构体)
 ******************************************************************************************/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_dt7_t *rc_dt7)
{
    if (sbus_buf == NULL || rc_dt7 == NULL) return;

    rc_dt7->rc_dt7.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    rc_dt7->rc_dt7.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    rc_dt7->rc_dt7.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;
    rc_dt7->rc_dt7.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    rc_dt7->rc_dt7.sw_l = ((sbus_buf[5] >> 4) & 0x0003);
    rc_dt7->rc_dt7.sw_r = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_dt7->mouse_dt7.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc_dt7->mouse_dt7.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc_dt7->mouse_dt7.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc_dt7->mouse_dt7.press_l = sbus_buf[12];
    rc_dt7->mouse_dt7.press_r = sbus_buf[13];
    rc_dt7->key_dt7.v = sbus_buf[14] | (sbus_buf[15] << 8);
    rc_dt7->rc_dt7.wheel = sbus_buf[16] | (sbus_buf[17] << 8);

    // 归一化到 [-660,660]
    rc_dt7->rc_dt7.ch[0] -= RC_CH_VALUE_OFFSET_DT7;
    rc_dt7->rc_dt7.ch[1] -= RC_CH_VALUE_OFFSET_DT7;
    rc_dt7->rc_dt7.ch[2] -= RC_CH_VALUE_OFFSET_DT7;
    rc_dt7->rc_dt7.ch[3] -= RC_CH_VALUE_OFFSET_DT7;

    rc_dt7->last_update_tick = osKernelSysTick();
}

/******************************************************************************************
 *                                   VT13 数据解析 (原VT13保留，适配新结构体)
 ******************************************************************************************/
static void RC_Data_Parse(volatile const uint8_t *p_frame, RC_vt13_t *rc_vt13)
{
    remote_raw_t *raw = (remote_raw_t *)p_frame;

    if (raw->sof_1 != 0xA9 || raw->sof_2 != 0x53) return;
    if (RC_CRC16_Check((uint8_t *)p_frame, RC_FRAME_LENGTH_VT13 - 2) != raw->crc16) return;

    // 映射摇杆与拨轮
    rc_vt13->rc_vt13.ch[0] = (int16_t)raw->ch_0 - RC_CH_VALUE_OFFSET_VT13;
    rc_vt13->rc_vt13.ch[1] = (int16_t)raw->ch_1 - RC_CH_VALUE_OFFSET_VT13;
    rc_vt13->rc_vt13.ch[2] = (int16_t)raw->ch_2 - RC_CH_VALUE_OFFSET_VT13;
    rc_vt13->rc_vt13.ch[3] = (int16_t)raw->ch_3 - RC_CH_VALUE_OFFSET_VT13;
    rc_vt13->rc_vt13.wheel = (int16_t)raw->wheel - RC_CH_VALUE_OFFSET_VT13;

    // 映射按键与挡位
    rc_vt13->rc_vt13.sw       = (uint8_t)raw->mode_sw;
    rc_vt13->rc_vt13.pause    = (uint8_t)raw->btn_pause;
    rc_vt13->rc_vt13.custom_l = (uint8_t)raw->btn_custom_l;
    rc_vt13->rc_vt13.custom_r = (uint8_t)raw->btn_custom_r;
    rc_vt13->rc_vt13.trigger  = (uint8_t)raw->btn_trigger;

    // 映射鼠标
    rc_vt13->mouse_vt13.x = raw->mouse_x;
    rc_vt13->mouse_vt13.y = raw->mouse_y;
    rc_vt13->mouse_vt13.z = raw->mouse_z;
    rc_vt13->mouse_vt13.press_l = (uint8_t)raw->mouse_left;
    rc_vt13->mouse_vt13.press_r = (uint8_t)raw->mouse_right;
    rc_vt13->mouse_vt13.press_m = (uint8_t)raw->mouse_middle;

    // 映射键盘
    rc_vt13->key_vt13.v = raw->key_v;

    rc_vt13->last_update_tick = osKernelSysTick();
}

/******************************************************************************************
 *                                   DT7 串口中断服务函数 USART3_IRQHandler (原DT7保留)
 ******************************************************************************************/
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM_DT7 - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM_DT7;
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH_DT7)
            {
                sbus_to_rc(sbus_rx_buf[0], &remote_ctrl.dt7);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM_DT7 - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM_DT7;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH_DT7)
            {
                sbus_to_rc(sbus_rx_buf[1], &remote_ctrl.dt7);
            }
        }
    }
}

/******************************************************************************************
 *                                   VT13/自定义控制器 串口中断服务函数 USART6_IRQHandler（扩展兼容）
 ******************************************************************************************/
void USART6_IRQHandler(void)
{
    if (huart6.Instance->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        uint16_t rx_len;
        uint8_t current_mem = (hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) ? 1 : 0;

        __HAL_DMA_DISABLE(&hdma_usart6_rx);

        // 计算实际接收长度，区分VT13帧（21字节）和自定义帧（39字节）
        if (current_mem == 0) {
            rx_len = CUSTOM_CTRL_FRAME_TOTAL_LEN - hdma_usart6_rx.Instance->NDTR;
        } else {
            rx_len = RC_RX_BUF_SIZE_VT13 - hdma_usart6_rx.Instance->NDTR;
        }

        // 重置缓冲区NDTR值
        if (current_mem == 0) {
            hdma_usart6_rx.Instance->NDTR = CUSTOM_CTRL_FRAME_TOTAL_LEN;
        } else {
            hdma_usart6_rx.Instance->NDTR = RC_RX_BUF_SIZE_VT13;
        }

        // 切换双缓冲
        if (current_mem == 0) {
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
        } else {
            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
        }

        __HAL_DMA_ENABLE(&hdma_usart6_rx);

        // 分支1：VT13帧（21字节）→ 原有解析逻辑
        if (rx_len == RC_FRAME_LENGTH_VT13) {
            RC_Data_Parse(rc_rx_buf[current_mem], &remote_ctrl.vt13);
        }
        // 分支2：自定义控制器帧（39字节）→ 新增解析逻辑
        else if (rx_len == CUSTOM_CTRL_FRAME_TOTAL_LEN) {
            uint8_t *frame = (current_mem == 0) ? custom_rx_buf[0] : custom_rx_buf[1];

            // 步骤1：帧头校验（SOF、CRC8、数据长度）
            if (!CustomCtrl_FrameHeader_Check(frame)) return;

            // 步骤2：命令码校验（必须为0x0302）
            uint16_t cmd_id = (frame[6] << 8) | frame[5]; // 命令码小端存储
            if (cmd_id != CUSTOM_CTRL_CMD_ID) return;

            // 步骤3：整包CRC16校验（校验范围：帧头+命令码+数据段）
            uint16_t calc_crc16 = RC_CRC16_Check((uint8_t *)frame, CUSTOM_CTRL_FRAME_TOTAL_LEN - 2);
            uint16_t recv_crc16 = (frame[38] << 8) | frame[37]; // 帧尾2字节为CRC16
            if (calc_crc16 != recv_crc16) return;

            // 步骤4：解析数据并存入结构体
            CustomCtrl_Data_Parse(frame, &remote_ctrl.custom);
        }
    }
}

/******************************************************************************************
 *                                   DT7 初始化 (原DT7保留)
 ******************************************************************************************/
void RC_Init_DT7(void)
{
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM_DT7;
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

/******************************************************************************************
 *                                   VT13 初始化 (原VT13保留)
 ******************************************************************************************/
void RC_Init_VT13(void)
{
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rc_rx_buf[0]);
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rc_rx_buf[1]);
    hdma_usart6_rx.Instance->NDTR = RC_RX_BUF_SIZE_VT13;
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

/******************************************************************************************
 *                                   自定义控制器 接收初始化（适配VT13图传链路UART6）
 ******************************************************************************************/
void RC_Init_Custom(void) {
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    // 配置双缓冲：M0=VT13缓冲，M1=自定义控制器缓冲
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rc_rx_buf[0]);    // VT13缓冲0
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(custom_rx_buf[0]);// 自定义缓冲0
    hdma_usart6_rx.Instance->NDTR = RC_RX_BUF_SIZE_VT13;         // 初始缓冲大小为VT13帧长
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);          // 使能双缓冲
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

/******************************************************************************************
 *                                   一键初始化双遥控器+自定义控制器
 ******************************************************************************************/
void RC_Init(void)
{
    RC_Init_DT7();
    RC_Init_VT13();
    RC_Init_Custom(); // 新增：初始化自定义控制器接收
}

/******************************************************************************************
 *                                   禁用函数 (原文件保留)
 ******************************************************************************************/
void RC_Unable_DT7(void)  { __HAL_UART_DISABLE(&huart3); }
void RC_Unable_VT13(void) { __HAL_UART_DISABLE(&huart6); }

/******************************************************************************************
 *                                   获取总遥控器句柄 (核心！给上层调用)
 ******************************************************************************************/
const RC_ctrl_t *RC_Get_Handle(void)
{
    return &remote_ctrl;
}