/**
 ****************************************************************************************************
 * @file        exti.c
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/EXTI/exti.h"

/**
 * @brief       �ⲿ�жϳ�ʼ������
 * @param       ��
 * @retval      ��
 */
void extix_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    KEY0_INT_GPIO_CLK_ENABLE();                              /* KEY0ʱ��ʹ�� */
    KEY1_INT_GPIO_CLK_ENABLE();                                 /* KEY1ʱ��ʹ�� */
    WKUP_INT_GPIO_CLK_ENABLE();                              /* WKUPʱ��ʹ�� */

    gpio_init_struct.Pin = KEY0_INT_GPIO_PIN;                /* KEY0���� */
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* �½��ش��� */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* ���� */
    HAL_GPIO_Init(KEY0_INT_GPIO_PORT, &gpio_init_struct);    /* KEY0���ų�ʼ�� */
 
    gpio_init_struct.Pin = KEY1_INT_GPIO_PIN;                /* KEY1���� */
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            /* �½��ش��� */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* ���� */
    HAL_GPIO_Init(KEY1_INT_GPIO_PORT, &gpio_init_struct);    /* KEY1���ų�ʼ�� */
    
    gpio_init_struct.Pin = WKUP_INT_GPIO_PIN;                /* WKUP���� */
    gpio_init_struct.Mode = GPIO_MODE_IT_RISING;             /* �����ش��� */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                   /* ���� */
    HAL_GPIO_Init(WKUP_INT_GPIO_PORT, &gpio_init_struct);    /* WKUP���ų�ʼ�� */

    HAL_NVIC_SetPriority(WKUP_INT_IRQn, 2, 0);               /* ��ռ���ȼ�Ϊ2�������ȼ�Ϊ0 */
    HAL_NVIC_EnableIRQ(WKUP_INT_IRQn);                       /* ʹ���ж���0 */
    
    HAL_NVIC_SetPriority(KEY0_INT_IRQn, 2, 1);               /* ��ռ���ȼ�Ϊ2�������ȼ�Ϊ1 */
    HAL_NVIC_EnableIRQ(KEY0_INT_IRQn);                       /* ʹ���ж���3 */
    
    HAL_NVIC_SetPriority(KEY1_INT_IRQn, 2, 2);               /* ��ռ���ȼ�Ϊ2�������ȼ�Ϊ2 */
    HAL_NVIC_EnableIRQ(KEY1_INT_IRQn);                       /* ʹ���ж���7 */
}


/**
 * @brief       �ⲿ�жϷ������
 * @param       ��
 * @retval      ��
 */
void WKUP_INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(WKUP_INT_GPIO_PIN);   /* �����жϴ����ú��� */
}

void KEY0_INT_IRQHandler(void)
{
    /* �����жϴ����ú��� */
    HAL_GPIO_EXTI_IRQHandler(KEY0_INT_GPIO_PIN); 
}

void KEY1_INT_IRQHandler(void)
{
    /* �����жϴ����ú��� */
    HAL_GPIO_EXTI_IRQHandler(KEY1_INT_GPIO_PIN); 
}


/**
 * @brief       GPIO�����ػص�����
 * @param       GPIO_Pin: �ж����ź�
 * @note        ��HAL�������е��ⲿ�жϷ�����������ô˺���
 * @retval      ��
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    /** �������˴�Ϊ�˷���ʹ������ʱ������ʵ�ʴ����н�ֹ
     *���жϷ������е����κ�delay֮�����ʱ���������� 
     */
    delay(2);     
    if (WK_UP == 1)            /* WK_UP�ж� */
    {
        LED1_TOGGLE();         /* LED1 ״̬ȡ�� */  
    }    
}


/**
 * @brief       GPIO�½��ػص�����
 * @param       GPIO_Pin: �ж����ź�
 * @note        ��HAL�������е��ⲿ�жϷ�����������ô˺���
 * @retval      ��
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    /** �������˴�Ϊ�˷���ʹ������ʱ������ʵ�ʴ����н�ֹ
     *���жϷ������е����κ�delay֮�����ʱ���������� 
     */
    delay(2);     
    if (KEY0 == 0)             /* KEY0�ж� */
    {
        BEEP_TOGGLE();      /* ������ ״̬ȡ�� */
    }
    else if(KEY1 == 0)        /* KEY1�ж� */
    {
        LED0_TOGGLE();        /*  LED0 ״̬ȡ�� */
    }
}

