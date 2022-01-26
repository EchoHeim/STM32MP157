/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       ���ڳ�ʼ�����룬֧��printf
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
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* ����4��GPIO���Ŷ����ʱ��ʹ�� */
#define USART_TX_GPIO_PORT                  GPIOG
#define USART_TX_GPIO_PIN                   GPIO_PIN_11
#define USART_TX_GPIO_AF                    GPIO_AF6_UART4
#define USART_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_2
#define USART_RX_GPIO_AF                    GPIO_AF8_UART4
#define USART_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

/* ����4���жϺź��жϷ��������壬�Լ�ʹ�ܴ���4ʱ�� */
#define USART_UX                            UART4
#define USART_UX_IRQn                       UART4_IRQn
#define USART_UX_IRQHandler                 UART4_IRQHandler
#define USART_UX_CLK_ENABLE()               do{ __HAL_RCC_UART4_CLK_ENABLE(); }while(0)   /* UART4 ʱ��ʹ�� */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* �����������ֽ��� 200 */
#define USART_EN_RX     1                       /* ʹ�ܣ�1��/��ֹ��0������4���� */
#define RXBUFFERSIZE    1                       /* �����С */
//extern uint8_t g_rx_buffer;
extern UART_HandleTypeDef g_uart4_handle;       /* UART4��� */

//extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* ���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
extern uint16_t g_usart_rx_sta;                 /* ����״̬��� */
//extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL��USART����Buffer */

void usart_init(uint32_t baudrate);             /* ���ڳ�ʼ���������� */
#endif

