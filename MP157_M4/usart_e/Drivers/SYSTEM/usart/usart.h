/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       串口初始化代码，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32MP1开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200505
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* 串口4的GPIO引脚定义和时钟使能 */
#define USART_TX_GPIO_PORT                  GPIOG
#define USART_TX_GPIO_PIN                   GPIO_PIN_11
#define USART_TX_GPIO_AF                    GPIO_AF6_UART4
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* 发送引脚时钟使能 */

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_2
#define USART_RX_GPIO_AF                    GPIO_AF8_UART4
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* 接收引脚时钟使能 */

/* 串口4的中断号和中断服务函数定义，以及使能串口4时钟 */
#define USART_UX                            UART4
#define USART_UX_IRQn                       UART4_IRQn
#define USART_UX_IRQHandler                 UART4_IRQHandler
#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_UART4_CLK_ENABLE(); }while(0)   /* UART4 时钟使能 */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* 定义最大接收字节数 200 */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口4接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */
//extern uint8_t g_rx_buffer;
extern UART_HandleTypeDef g_uart4_handle;       /* UART4句柄 */

//extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
//extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL库USART接收Buffer */

void usart_init(uint32_t baudrate);             /* 串口初始化函数声明 */
#endif

