/**
 ****************************************************************************************************
 * @file        exti.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       �ⲿ�ж� ��������
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


#ifndef __EXTI_H
#define __EXTI_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* ���� �� �жϱ�� & �жϷ����� ���� */ 

#define KEY0_INT_GPIO_PORT              GPIOG
#define KEY0_INT_GPIO_PIN               GPIO_PIN_3
#define KEY0_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* PG��ʱ��ʹ�� */
#define KEY0_INT_IRQn             		EXTI3_IRQn
#define KEY0_INT_IRQHandler       		EXTI3_IRQHandler

#define KEY1_INT_GPIO_PORT              GPIOH
#define KEY1_INT_GPIO_PIN               GPIO_PIN_7
#define KEY1_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)   /* PG��ʱ��ʹ�� */
#define KEY1_INT_IRQn             		EXTI7_IRQn
#define KEY1_INT_IRQHandler       		EXTI7_IRQHandler

#define WKUP_INT_GPIO_PORT              GPIOA
#define WKUP_INT_GPIO_PIN               GPIO_PIN_0
#define WKUP_INT_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */
#define WKUP_INT_IRQn                   EXTI0_IRQn
#define WKUP_INT_IRQHandler             EXTI0_IRQHandler

/******************************************************************************************/

void extix_init(void); //�ⲿ�жϳ�ʼ��

#endif

















