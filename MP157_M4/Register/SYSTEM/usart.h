/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */
#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"

#define USARTx UART8

/*******************************************************************************************************/


#define USART_REC_LEN   200                     /* 定义最大接收字节数 200 */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口1接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */

#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3

extern UART_HandleTypeDef g_uart4_handle;       /* UART句柄 */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL库USART接收Buffer */

extern void gpio_peripheral(uint32_t gpio, uint32_t Mode,uint32_t Pull, uint32_t Speed,uint32_t AF_Mode);
extern void gpio_peripheral_K(uint32_t gpio, uint32_t mode, int pullup);

extern void enable_pclock(uint32_t periph_base);

void usart_init(uint32_t baudrate);             /* 串口初始化函数 */
#endif

