/**
 ****************************************************************************************************
 * @file        wdg.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-06
 * @brief       ���ڿ��Ź� ��������
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
 * V1.0 20200506
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "wwdg.h"
#include "gpio.h"


WWDG_HandleTypeDef g_wwdg_handle;     /* ���ڿ��Ź���� */

/**
 * @brief       ��ʼ�����ڿ��Ź�
 * @param       tr: T[6:0],������ֵ
 * @param       tw: W[6:0],����ֵ
 * @note        fprer:��Ƶϵ����WDGTB��,��Χ:0~7,��ʾ2^WDGTB��Ƶ
 *              Fwwdg=PCLK1(APB1)/(4096*2^fprer). һ��PCLK1=104.5Mhz
 *              ���ʱ��=(4096*2^fprer)*(tr-0X3F)/PCLK1
 *              ����fprer=4,tr=7f,PCLK3=104.5Mhz
 *              �����ʱ��=4096*16*64/104.5Mhz=40.13ms
 * @retval      ��
 */ 
void wwdg_init(uint8_t tr, uint8_t wr, uint32_t fprer)
{
//    g_wwdg_handle.Instance = WWDG1;
//    g_wwdg_handle.Init.Prescaler = fprer;  /* ���÷�Ƶϵ�� */
//    g_wwdg_handle.Init.Window = wr;        /* ���ô���ֵ */
//    g_wwdg_handle.Init.Counter = tr;       /* ���ü�����ֵ */
//    g_wwdg_handle.Init.EWIMode = WWDG_EWI_ENABLE;/* ʹ�ܴ��ڿ��Ź���ǰ�����ж� */
    
    RCC->MC_APB1ENSETR = RCC_MC_APB1ENSETR_WWDG1EN; /* ʹ�ܴ��ڿ��Ź�ʱ�� */
    
    HAL_NVIC_SetPriority(WWDG1_IRQn,2,3);     /* ��ռ���ȼ�2�������ȼ�Ϊ3 */
    HAL_NVIC_EnableIRQ(WWDG1_IRQn);           /* ʹ�ܴ��ڿ��Ź��ж� */

    /* Set WWDG Counter */
    WRITE_REG(WWDG1->CR, (WWDG_CR_WDGA | tr));

    /* Set WWDG Prescaler and Window */
    WRITE_REG(WWDG1->CFR, (WWDG_CFR_EWI | fprer | wr));
}


/**
 * @brief       ���ڿ��Ź��жϷ������
 * @param       ��
 * @retval      ��
 */
void WWDG1_IRQHandler(void)
{
    if((WWDG1->CFR & WWDG_IT_EWI) == WWDG_IT_EWI)
    {
        if((WWDG1->SR & WWDG_FLAG_EWIF) == WWDG_FLAG_EWIF)
        {
            

        WWDG1->SR = ~(WWDG_SR_EWIF);/* Clear the WWDG Early Wakeup flag */

      /* Early Wakeup callback */
      WRITE_REG(WWDG1->CR, (0x7F)); /* ���´��ڿ��Ź�ֵ */
        LED1_TOGGLE();                   /* �̵���˸ */
        }
    }
}



















