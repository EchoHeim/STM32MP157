/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
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
#include "sys.h"
#include "usart.h"

/* ���ʹ��os,����������ͷ�ļ�����. */
#if SYS_SUPPORT_OS
#include "includes.h"   /* os ʹ�� */
#endif

/******************************************************************************************/

/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1
#pragma import(__use_no_semihosting)

/* ���HAL��ʹ��ʱ, ĳЩ������ܱ����bug */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ���HAL��ʹ��ʱ, ĳЩ������ܱ����bug */
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

/* FILE is typedef�� d in stdio.h. */
FILE __stdout;

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */

int fputc(int ch, FILE *f)
{
    while ((USARTx->ISR & 0X40) == 0);    // �ȴ���һ���ַ�������� 

    USARTx->TDR = (uint8_t)ch;            // ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� 
    return ch;
}

#endif

/***********************************************END*******************************************/

#if USART_EN_RX     /* ���ʹ���˽��� */

/* ���ջ���, ���USART_REC_LEN���ֽ�. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

UART_HandleTypeDef g_usart3_handle;    /* UART��� */

/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RXBUFFERSIZE];    /* HAL��ʹ�õĴ��ڽ��ջ��� */

/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart_init(uint32_t baudrate)
{
    enable_pclock((uint32_t)UART8);     // USART ʱ��ʹ�� 

//    HAL_UART_MspInit(&g_usart3_handle);
    
//    gpio_peripheral_K(GPIO('E', 0), GPIO_FUNCTION(8), 1);
//    gpio_peripheral_K(GPIO('E', 1), GPIO_FUNCTION(8), 1);

    gpio_peripheral(GPIO('E', 0),GPIO_MODE_AF_PP,GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0x08);
    gpio_peripheral(GPIO('E', 1),GPIO_MODE_AF_PP,GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0x08);

    /* ����UART4ʱ��Դ=PLL4Q=74.25MHz  */
    WRITE_REG(RCC->UART78CKSELR, (0x02));

//#if USART_EN_RX
//    HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          /* ʹ��USART3�ж�ͨ�� */
//    HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);                  /* ��ռ���ȼ�3�������ȼ�3 */
//#endif

    UART8->BRR = (0x22C);
    UART8->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE);
    
//    HAL_UART_Init(&g_usart3_handle);                        /* HAL_UART_Init()��ʹ��UART1 */
    
    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
 //   HAL_UART_Receive_IT(&g_usart3_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}


/**
 * @brief       Rx����ص�����
 * @param       huart: UART�������ָ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USARTx)                             /* ����Ǵ���3 */
    {
        if((g_usart_rx_sta & 0x8000) == 0)                    /* ����δ��� */
        {
            if(g_usart_rx_sta & 0x4000)                       /* ���յ���0x0d */
            {
                if(g_rx_buffer[0] != 0x0a)
                {
                    g_usart_rx_sta = 0;                       /* ���մ���,���¿�ʼ */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;                 /* ��������� */
                }
            }
            else                                              /* ��û�յ�0X0D */
            {
                if(g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if(g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;                   /* �������ݴ���,���¿�ʼ���� */
                    }
                }
            }
        }
    }
}

/**
 * @brief       ����3�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{ 
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;
    
#if SYS_SUPPORT_OS                        /* ʹ��OS */
    OSIntEnter();    
#endif

    HAL_UART_IRQHandler(&g_usart3_handle); /* ����HAL���жϴ����ú��� */

    timeout = 0;
    while (HAL_UART_GetState(&g_usart3_handle) != HAL_UART_STATE_READY) /* �ȴ����� */
    {
        timeout++;                       /* ��ʱ���� */
        if(timeout > maxDelay)
        {
            break;
        }
    }
     
    timeout=0;
    
    /* һ�δ������֮�����¿����жϲ�����RxXferCountΪ1 */
    while (HAL_UART_Receive_IT(&g_usart3_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE) != HAL_OK)
    {
        timeout++;                  /* ��ʱ���� */
        if (timeout > maxDelay)
        {
            break;
        }
    }

#if SYS_SUPPORT_OS                  /* ʹ��OS */
    OSIntExit();
#endif

}

#endif
