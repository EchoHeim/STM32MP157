/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-05-05
 * @brief       串口初始化代码(一般是串口4)，支持printf
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
 * V1.0 20200505
 * 第一次发布
 *
 ****************************************************************************************************
 */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"

/* 如果使用os,则包括下面的头文件即可. */
#if SYS_SUPPORT_OS
#include "includes.h"   /* os 使用 */
#endif


/******************************************************************************************/

/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#pragma import(__use_no_semihosting)

/* 解决HAL库使用时, 某些情况可能报错的bug */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 解决HAL库使用时, 某些情况可能报错的bug */
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

/* FILE is typedef’ d in stdio.h. */
FILE __stdout;

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART_UX->ISR & 0X40) == 0);    /* 等待上一个字符发送完成 */

    USART_UX->TDR = (uint8_t)ch;            /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}

#endif

/***********************************************END*******************************************/



/* 接收缓冲, 最大USART_REC_LEN个字节. */
//uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/
uint16_t g_usart_rx_sta = 0;

//uint8_t g_rx_buffer[RXBUFFERSIZE];    /* HAL库使用的串口接收缓冲 */
uint8_t g_rx_buffer;
UART_HandleTypeDef g_uart4_handle;    /* UART句柄 */


/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart_init(uint32_t baudrate)
{
    g_uart4_handle.Instance = USART_UX;                    /* USART4 */
    g_uart4_handle.Init.BaudRate = baudrate;               /* 波特率 */
    g_uart4_handle.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    g_uart4_handle.Init.StopBits = UART_STOPBITS_1;        /* 一个停止位 */
    g_uart4_handle.Init.Parity = UART_PARITY_NONE;         /* 无奇偶校验位 */
    g_uart4_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;   /* 无硬件流控 */
    g_uart4_handle.Init.Mode = UART_MODE_TX_RX;            /* 收发模式 */
    HAL_UART_Init(&g_uart4_handle);                        /* HAL_UART_Init()会使能UART4 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
   //HAL_UART_Receive_IT(&g_uart4_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    RCC_PeriphCLKInitTypeDef rcc_periphclk_init_struct;
    
    if(huart->Instance == UART4)                                   /* 如果是串口4，进行串口4 MSP初始化 */
    {
        USART_UX_CLK_ENABLE();                                      /* USART4时钟使能 */
        USART_TX_GPIO_CLK_ENABLE();                                 /* 发送引脚时钟使能 */
        USART_RX_GPIO_CLK_ENABLE();                                 /* 接收引脚时钟使能 */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;                   /* 指定TX引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;              /* 复用为UART4 */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);       /* 初始化发送引脚 */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;                   /* 指定RX引脚 */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;              /* 复用为UART4 */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);       /* 初始化接收引脚 */
        
        /* 设置UART4时钟源=PLL4Q=74.25MHz  */
        rcc_periphclk_init_struct.Uart24ClockSelection = RCC_UART24CLKSOURCE_PLL4;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periphclk_init_struct);


        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                          /* 使能USART4中断通道 */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);                  /* 抢占优先级3，子优先级3 */

    }
}

/**
 * @brief       Rx传输回调函数
 * @param       huart: UART4句柄类型指针
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  HAL_UART_Transmit(&g_uart4_handle,&g_rx_buffer,1,0);
  HAL_UART_Receive_IT(&g_uart4_handle,&g_rx_buffer,1);
}

/**
 * @brief       串口4中断服务函数
 * @param       无
 * @retval      无
 */
void USART_UX_IRQHandler(void)
{ 
    //printf("f\r\n");
    HAL_UART_IRQHandler(&g_uart4_handle);
}

