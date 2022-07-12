/**
 ****************************************************************************************************
 * @file        delay.h
 * @author      ����ԭ��Linux�Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-04
 * @brief       �ṩ���õ���ʱ����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32H750������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200504
 * ��һ�η���
 * V2.0 20200505
 * ���systick��ʼ����Ϣ����Ӹ߾�����ʱ����delay_us��delay_ms�Ⱥ���
 *
 ****************************************************************************************************
 */

#ifndef __DELAY_H
#define __DELAY_H

#include "sys.h"

void delay_init(uint16_t sysclk); /* ��ʼ���ӳٺ��� */
void delay_ms(uint16_t nms);      /* ��ʱnms */
void delay_us(uint32_t nus);      /* ��ʱnus */

void delay_short(volatile unsigned int n);
void delay(volatile unsigned int n);

#endif

