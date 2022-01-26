/**
 ****************************************************************************************************
 * @file        beep.h
 * @author      正点原子Linux团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-04
 * @brief       蜂鸣器 驱动代码
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
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __BEEP_H
#define __BEEP_H

#include "./SYSTEM/sys/sys.h"

/*****************************************************************************************************/
/* 引脚 定义 */

#define BEEP_GPIO_PORT                  GPIOC
#define BEEP_GPIO_PIN                   GPIO_PIN_7
#define BEEP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

/*****************************************************************************************************/


/* 蜂鸣器控制 */
#define BEEP(x)   do{ x ? \
                      HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

/* 蜂鸣器取反控制 */
#define BEEP_TOGGLE()  do{ HAL_GPIO_TogglePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN); }while(0)  /* BEEP = !BEEP */


void beep_init(void); //BEEP初始化函数
#endif















