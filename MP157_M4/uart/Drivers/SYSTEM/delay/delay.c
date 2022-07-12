/**
 ****************************************************************************************************
 * @file        delay.c
 * @author      ����ԭ��Linux�Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-04
 * @brief       �ṩ���õ���ʱ����
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
 * ��һ�η���
 *
 ****************************************************************************************************
 */
 
#include "./SYSTEM/Delay/delay.h"

/**
 * @brief        ��ʱ����ʱ����
 * @param - n    Ҫ��ʱѭ������(�ղ���ѭ��������ģʽ��ʱ)
 * @retval         ��
 */
void delay_short(volatile unsigned int n)
{
    while(n--){}
}

/**
 * @brief        ����ʱ����
 * @param - n    Ҫ��ʱ��ʱ��ѭ����
 * @retval         ��
 */
void delay(volatile unsigned int n)
{
    while(n--)
    {
        delay_short(0x7fff);
    }
}
