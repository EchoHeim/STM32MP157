/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      ����ԭ��Linux�Ŷ�(ALIENTEK)
 * @version     V1.1
 * @date        2020-05-04
 * @brief       ϵͳ��ʼ������(����ʱ������/�жϹ���/GPIO���õ�)
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
 * V1.0 20200504
 ****************************************************************************************************
 */

#ifndef __SYS_H
#define __SYS_H

#include "stm32mp1xx.h"
#include "stm32mp1xx_hal.h"
#include "core_cm4.h"

/**
 * SYS_SUPPORT_OS���ڶ���ϵͳ�ļ����Ƿ�֧��OS
 * 0,��֧��OS
 * 1,֧��OS
 */
#define SYS_SUPPORT_OS         0
#define      ON      1
#define      OFF     0

uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv);

#if defined(__clang__)                           /* ʹ��V6������(clang) */

void __attribute__((noinline)) WFI_SET(void);
void __attribute__((noinline)) INTX_DISABLE(void);
void __attribute__((noinline)) INTX_ENABLE(void);
void __attribute__((noinline)) MSR_MSP(uint32_t addr);

#elif defined (__CC_ARM)                    /* ʹ��V5������(ARMCC) */

/* ����Ϊ��ຯ�� */
void WFI_SET(void);                         /* ִ��WFIָ�� */
void INTX_DISABLE(void);                    /* �ر������ж� */
void INTX_ENABLE(void);                     /* ���������ж� */
void MSR_MSP(uint32_t addr);                /* ���ö�ջ��ַ */
#endif

#endif
