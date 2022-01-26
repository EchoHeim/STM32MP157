/**
 ****************************************************************************************************
 * @file        delay.h
 * @author      正点原子Linux团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-04
 * @brief       提供常用的延时函数
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32H750开发板
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

#ifndef __DELAY_H
#define __DELAY_H

void delay_short(volatile unsigned int n);
void delay(volatile unsigned int n);

#endif

