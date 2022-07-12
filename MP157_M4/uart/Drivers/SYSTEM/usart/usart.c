/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       ���ڳ�ʼ������(һ���Ǵ���4)��֧��printf
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
#include "./SYSTEM/usart/usart.h"
#include "klipper.h"

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
    while ((USART_UX->ISR & 0X40) == 0);    /* �ȴ���һ���ַ�������� */

    USART_UX->TDR = (uint8_t)ch;            /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}

#endif

/***********************************************END*******************************************/


/* ���ջ���, ���USART_REC_LEN���ֽ�. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t g_usart_rx_sta = 0;

//uint8_t g_rx_buffer[RXBUFFERSIZE];    /* HAL��ʹ�õĴ��ڽ��ջ��� */

uint8_t g_rx_buffer;
UART_HandleTypeDef g_uart4_handle;    /* UART��� */



/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart_init(uint32_t baudrate)
{
 //   RCC_PeriphCLKInitTypeDef rcc_periphclk_init_struct;
    
//    g_uart4_handle.Instance = USART_UX;                    /* USART4 */
//    g_uart4_handle.Init.BaudRate = baudrate;               /* ������ */
//    g_uart4_handle.Init.WordLength = UART_WORDLENGTH_8B;   /* �ֳ�Ϊ8λ���ݸ�ʽ */
//    g_uart4_handle.Init.StopBits = UART_STOPBITS_1;        /* һ��ֹͣλ */
//    g_uart4_handle.Init.Parity = UART_PARITY_NONE;         /* ����żУ��λ */
//    g_uart4_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* ��Ӳ������ */
//    g_uart4_handle.Init.Mode = UART_MODE_TX_RX;            /* �շ�ģʽ */
    
    enable_pclock((uint32_t)USART_UX);

    gpio_peripheral(GPIO('E', 0), GPIO_FUNCTION(8), 1);
    gpio_peripheral(GPIO('E', 1), GPIO_FUNCTION(8), 0);

//    WRITE_REG(USART_UX->BRR, (0x22C));

//    uint32_t pclk = get_pclock_frequency((uint32_t)USART_UX);
//    uint32_t div = DIV_ROUND_CLOSEST(pclk, 115200);
//    USART_UX->BRR = (((div / 16) << 4)
//                 | ((div % 16) << USART_BRR_LPUART_Pos));
//    
    
        /* ����UART4ʱ��Դ=PLL4Q=74.25MHz  */
 //       WRITE_REG(RCC->UART78CKSELR, (0x02));
//        rcc_periphclk_init_struct.Uart78ClockSelection = RCC_UART78CLKSOURCE_PLL4;
//        HAL_RCCEx_PeriphCLKConfig(&rcc_periphclk_init_struct);

#if 1
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          /* ʹ��USART4�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);                  /* ��ռ���ȼ�3�������ȼ�3 */
#endif

#if 0

  g_uart4_handle.Instance->CR1 &= ~USART_CR1_UE;

  // Set the UART Communication parameters 
  if (UART_SetConfig(&g_uart4_handle) == HAL_ERROR)
  {
    return ;
  }

//  if (g_uart4_handle.AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
//  {
//    UART_AdvFeatureConfig(&g_uart4_handle);
//  }

//   In asynchronous mode, the following bits must be kept cleared:
//  - LINEN and CLKEN bits in the USART_CR2 register,
//  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
  CLEAR_BIT(g_uart4_handle.Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(g_uart4_handle.Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  g_uart4_handle.Instance->CR1 |= USART_CR1_UE;
#endif

  USART_UX->BRR = (0x38B);
  USART_UX->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE);
  
#if 0
  uint32_t tickstart;
  
  /* Initialize the UART ErrorCode */
  g_uart4_handle.ErrorCode = HAL_UART_ERROR_NONE;

  /* Init tickstart for timeout managment*/
  tickstart = HAL_GetTick();

  /* Check if the Transmitter is enabled */
  if ((g_uart4_handle.Instance->CR1 & USART_CR1_TE) == USART_CR1_TE)
  {
    /* Wait until TEACK flag is set */
    if (UART_WaitOnFlagUntilTimeout(&g_uart4_handle, USART_ISR_TEACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Timeout occurred */
      return ;
    }
  }

  /* Check if the Receiver is enabled */
  if ((g_uart4_handle.Instance->CR1 & USART_CR1_RE) == USART_CR1_RE)
  {
    /* Wait until REACK flag is set */
    if (UART_WaitOnFlagUntilTimeout(&g_uart4_handle, USART_ISR_REACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Timeout occurred */
      return ;
    }
  }
#endif
  
  /* Initialize the UART State */
  g_uart4_handle.gState = HAL_UART_STATE_READY;
  g_uart4_handle.RxState = HAL_UART_STATE_READY;

  g_uart4_handle.Lock = HAL_UNLOCKED; 

  //  HAL_UART_Init(&g_uart4_handle);                        /* HAL_UART_Init()��ʹ��UART4 */

//    WRITE_REG(USART_UX->ISR,0x00C0);
//    WRITE_REG(USART_UX->CR1,0x000C);
//    
//    WRITE_REG(USART_UX->BRR,0x038B);
//    WRITE_REG(USART_UX->CR1,0x000D);
//    
//    UART4->BRR = (0x38B);
//    USART_UX->CR1 = 0x12D; //(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE);
    
//    USART_UX->ISR = 0x006000D0;
//    WRITE_REG(USART_UX->ISR,0x006010D0);
//    WRITE_REG(USART_UX->CR1,0x012D);
//    WRITE_REG(USART_UX->CR3,0x01);
 //   USART_UX->CR3 = 0x01;
    
    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
//    HAL_UART_Receive_IT(&g_uart4_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
    HAL_UART_Receive_IT(&g_uart4_handle,&g_rx_buffer,1);/* ���жϷ�ʽ���պ��� */
}

/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����
 * @retval      ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    RCC_PeriphCLKInitTypeDef rcc_periphclk_init_struct;
    
    if(huart->Instance == USART_UX)                                   /* ����Ǵ���4�����д���4 MSP��ʼ�� */
    {
        enable_pclock((uint32_t)USART_UX);
  //      USART_UX_CLK_ENABLE();                                      /* USART4ʱ��ʹ�� */
//        USART_TX_GPIO_CLK_ENABLE();                                 /* ��������ʱ��ʹ�� */
//        USART_RX_GPIO_CLK_ENABLE();                                 /* ��������ʱ��ʹ�� */

//        gpio_init_struct.Pin = USART_TX_GPIO_PIN;                   /* ָ��TX���� */
//        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
//        gpio_init_struct.Pull = GPIO_PULLUP;                        /* ���� */
//        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
//        gpio_init_struct.Alternate = USART_TX_GPIO_AF;              /* ����ΪUART4 */
//        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);       /* ��ʼ���������� */

//        gpio_init_struct.Pin = USART_RX_GPIO_PIN;                   /* ָ��RX���� */
//        gpio_init_struct.Alternate = USART_RX_GPIO_AF;              /* ����ΪUART4 */
//        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);       /* ��ʼ���������� */
        
        gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(8), 1);
        gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(6), 0);
        
//        gpio_peripheral_c(GPIO_Rx,GPIO_MODE_AF_PP,GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0x08);
//        gpio_peripheral_c(GPIO_Tx,GPIO_MODE_AF_PP,GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, 0x08);
        
        /* ����UART4ʱ��Դ=PLL4Q=74.25MHz  */
 //       WRITE_REG(RCC->UART78CKSELR, (0x02));
        
//        rcc_periphclk_init_struct.Uart78ClockSelection = RCC_UART78CLKSOURCE_PLL4;
//        HAL_RCCEx_PeriphCLKConfig(&rcc_periphclk_init_struct);

#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          /* ʹ��USART4�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);                  /* ��ռ���ȼ�3�������ȼ�3 */
#endif
    }
}

/**
 * @brief       Rx����ص�����
 * @param       huart: UART4�������ָ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Transmit(&g_uart4_handle,&g_rx_buffer,1,0);
  HAL_UART_Receive_IT(&g_uart4_handle,&g_rx_buffer,1);
}

/**
 * @brief       ����4�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{ 

    HAL_UART_IRQHandler(&g_uart4_handle); /* ����HAL���жϴ����ú��� */

}
