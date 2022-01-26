/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      正点原子Linux团队(ALIENTEK)
 * @version     V1.1
 * @date        2020-05-04
 * @brief       系统初始化代码(包括时钟配置/中断管理/GPIO设置等)
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
 * V1.0 20200504
 ****************************************************************************************************
 */

#ifndef __SYS_H
#define __SYS_H

#include "stm32mp1xx.h"
#include "stm32mp1xx_hal.h"
#include "core_cm4.h"

/**
 * SYS_SUPPORT_OS用于定义系统文件夹是否支持OS
 * 0,不支持OS
 * 1,支持OS
 */
#define SYS_SUPPORT_OS         0
#define      ON      1
#define      OFF     0

uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv);

#if defined(__clang__)                           /* 使用V6编译器(clang) */

void __attribute__((noinline)) WFI_SET(void);
void __attribute__((noinline)) INTX_DISABLE(void);
void __attribute__((noinline)) INTX_ENABLE(void);
void __attribute__((noinline)) MSR_MSP(uint32_t addr);

#elif defined (__CC_ARM)                    /* 使用V5编译器(ARMCC) */

/* 以下为汇编函数 */
void WFI_SET(void);                         /* 执行WFI指令 */
void INTX_DISABLE(void);                    /* 关闭所有中断 */
void INTX_ENABLE(void);                     /* 开启所有中断 */
void MSR_MSP(uint32_t addr);                /* 设置堆栈地址 */
#endif

#endif
