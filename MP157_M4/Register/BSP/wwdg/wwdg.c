/**
 ****************************************************************************************************
 * @file        wdg.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-06
 * @brief       窗口看门狗 驱动代码
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
 * V1.0 20200506
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "wwdg.h"
#include "gpio.h"


WWDG_HandleTypeDef g_wwdg_handle;     /* 窗口看门狗句柄 */

/**
 * @brief       初始化窗口看门狗
 * @param       tr: T[6:0],计数器值
 * @param       tw: W[6:0],窗口值
 * @note        fprer:分频系数（WDGTB）,范围:0~7,表示2^WDGTB分频
 *              Fwwdg=PCLK1(APB1)/(4096*2^fprer). 一般PCLK1=104.5Mhz
 *              溢出时间=(4096*2^fprer)*(tr-0X3F)/PCLK1
 *              假设fprer=4,tr=7f,PCLK3=104.5Mhz
 *              则溢出时间=4096*16*64/104.5Mhz=40.13ms
 * @retval      无
 */ 
void wwdg_init(uint8_t tr, uint8_t wr, uint32_t fprer)
{
//    g_wwdg_handle.Instance = WWDG1;
//    g_wwdg_handle.Init.Prescaler = fprer;  /* 设置分频系数 */
//    g_wwdg_handle.Init.Window = wr;        /* 设置窗口值 */
//    g_wwdg_handle.Init.Counter = tr;       /* 设置计数器值 */
//    g_wwdg_handle.Init.EWIMode = WWDG_EWI_ENABLE;/* 使能窗口看门狗提前唤醒中断 */
    
    RCC->MC_APB1ENSETR = RCC_MC_APB1ENSETR_WWDG1EN; /* 使能窗口看门狗时钟 */
    
    HAL_NVIC_SetPriority(WWDG1_IRQn,2,3);     /* 抢占优先级2，子优先级为3 */
    HAL_NVIC_EnableIRQ(WWDG1_IRQn);           /* 使能窗口看门狗中断 */

    /* Set WWDG Counter */
    WRITE_REG(WWDG1->CR, (WWDG_CR_WDGA | tr));

    /* Set WWDG Prescaler and Window */
    WRITE_REG(WWDG1->CFR, (WWDG_CFR_EWI | fprer | wr));
}


/**
 * @brief       窗口看门狗中断服务程序
 * @param       无
 * @retval      无
 */
void WWDG1_IRQHandler(void)
{
    if((WWDG1->CFR & WWDG_IT_EWI) == WWDG_IT_EWI)
    {
        if((WWDG1->SR & WWDG_FLAG_EWIF) == WWDG_FLAG_EWIF)
        {
            

        WWDG1->SR = ~(WWDG_SR_EWIF);/* Clear the WWDG Early Wakeup flag */

      /* Early Wakeup callback */
      WRITE_REG(WWDG1->CR, (0x7F)); /* 更新窗口看门狗值 */
        LED1_TOGGLE();                   /* 绿灯闪烁 */
        }
    }
}



















