#include "remote_control.h"
#include "bsp_CAN.h"
#include "detect_task.h"
#include "bsp_dwt.h"
#include "bsp_usart_idle.h"

RC_Type remote_control = {0};   
uint8_t RC_Data_Buffer[16] = {0}; 
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM]; 
uint32_t RC_DWT_Count = 0;                   
float RC_dt = 0;  

void Remote_Control_Init(UART_HandleTypeDef *huart)
{
    remote_control.RC_USART = huart;

    USART_IDLE_Init(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
    if (buff==NULL || rc==NULL)
        return;
    memcpy(RC_Data_Buffer, buff, 16);
    RC_dt = DWT_GetDeltaT(&RC_DWT_Count);
    rc->ch1 = (buff[0] | (buff[1] << 8)) & 0x07FF;
    rc->ch1 -= RC_CH_VALUE_OFFSET;
    rc->ch2 =((buff[1] >> 3)|(buff[2] << 5) ) & 0x07ff;
    rc->ch2 -= RC_CH_VALUE_OFFSET;
    rc->ch3 = ((buff[2]>>6) | (buff[3]<<2) | (buff[4] << 10))& 0x07ff;
    rc->ch3-=RC_CH_VALUE_OFFSET;
    rc->ch4=((buff[4] >> 1)| buff[5] << 7) & 0x07ff;
    rc->ch4-=RC_CH_VALUE_OFFSET;
    rc->switch_left  = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->switch_right = (buff[5] >> 4) & 0x0003;
    rc->mouse.x = buff[6] | (buff[7] << 8);
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);
    rc->mouse.press_left  = buff[12];
    rc->mouse.press_right = buff[13];
    rc->key_code = buff[14] | (buff[15] << 8);

    RC_Update = 1;
}

void Solve_RC_Lost(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}//掉线保护

void Solve_RC_Data_Error(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

uint8_t RC_Data_is_Error(void)
{
    // 检查4个通道值是否在合理范围
    if (abs(remote_control.ch1) > 1000) goto error;
    if (abs(remote_control.ch2) > 1000) goto error;
    if (abs(remote_control.ch3) > 1000) goto error;
    if (abs(remote_control.ch4) > 1000) goto error;

    // 检查开关值
    if (remote_control.switch_left  == 0) goto error;
    if (remote_control.switch_right == 0) goto error;

    return 0;  // 数据正常

error:
    remote_control.ch1 = 0;
    remote_control.ch2 = 0;
    remote_control.ch3 = 0;
    remote_control.ch4 = 0;
    remote_control.switch_left  = 0;
    remote_control.switch_right = 0;
    remote_control.mouse.x = 0;
    remote_control.mouse.y = 0;
    remote_control.mouse.z = 0;
    remote_control.mouse.press_left  = 0;
    remote_control.mouse.press_right = 0;
    remote_control.key_code = 0;

    return 1;  // 数据异常
}