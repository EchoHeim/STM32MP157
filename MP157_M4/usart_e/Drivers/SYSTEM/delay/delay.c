/**
 ****************************************************************************************************
 * @file        delay.c
 * @author      正点原子Linux团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-04
 * @brief       提供常用的延时函数
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
 
#include "./SYSTEM/Delay/delay.h"

/**
 * @brief        短时间延时函数
 * @param - n    要延时循环次数(空操作循环次数，模式延时)
 * @retval         无
 */
void delay_short(volatile unsigned int n)
{
    while(n--){}
}

/**
 * @brief        长延时函数
 * @param - n    要延时的时间循环数
 * @retval         无
 */
void delay(volatile unsigned int n)
{
    while(n--)
    {
        delay_short(0x7fff);
    }
}
