/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32MP1������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200505
 * ��һ�η���
 *
 ****************************************************************************************************
 */
#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"

#if 0
/*******************************************************************************************************/
/* ����3��GPIO */

#define USART_TX_GPIO_PORT                  GPIOB
#define USART_TX_GPIO_PIN                   GPIO_PIN_10
#define USART_TX_GPIO_AF                    GPIO_AF7_USART3
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_12
#define USART_RX_GPIO_AF                    GPIO_AF8_USART3
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_UX                            USART3
#define USART_UX_IRQn                       USART3_IRQn
#define USART_UX_IRQHandler                 USART3_IRQHandler

#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)   /* USART3 ʱ��ʹ�� */

/*******************************************************************************************************/

#else
/* ����4��GPIO */

#define USART_TX_GPIO_PORT                  GPIOG
#define USART_TX_GPIO_PIN                   GPIO_PIN_11
#define USART_TX_GPIO_AF                    GPIO_AF6_UART4
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_2
#define USART_RX_GPIO_AF                    GPIO_AF8_UART4
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_UX                            UART4
#define USART_UX_IRQn                       UART4_IRQn
#define USART_UX_IRQHandler                 UART4_IRQHandler

#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_UART4_CLK_ENABLE(); }while(0)   /* USART3 ʱ��ʹ�� */

#endif
/*******************************************************************************************************/


#define USART_REC_LEN   200                     /* �����������ֽ��� 200 */
#define USART_EN_RX     1                       /* ʹ�ܣ�1��/��ֹ��0������1���� */
#define RXBUFFERSIZE    1                       /* �����С */

extern UART_HandleTypeDef g_uart4_handle;       /* UART��� */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* ���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
extern uint16_t g_usart_rx_sta;                 /* ����״̬��� */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL��USART����Buffer */


void usart_init(uint32_t baudrate);             /* ���ڳ�ʼ������ */
#endif

