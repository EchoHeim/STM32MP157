#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./SYSTEM/usart/usart.h"
#include "klipper.h"

void delayShort(volatile unsigned int n)
{
    while(n--){}
}


void D_elay(volatile unsigned int n)
{
    while(n--)
    {
        delayShort(0x7fff);
    }
}

#define RCC_MC_AHB4ENSETR        *((volatile unsigned int *)(RCC_BASE + 0XAA8))            /* RCC_MC_AHB4ENSETR寄存器的地址为0x50000AA8 */
#define GPIOI_MODER              *((volatile unsigned int *)(GPIOI_BASE + 0x0000))        /* GPIOI_MODER的地址为 0x5000A000 */
#define GPIOI_OTYPER              *((volatile unsigned int *)(GPIOI_BASE + 0x0004))    /* GPIOI_OTYPER的地址为0x5000A004 */
#define GPIOI_OSPEEDR              *((volatile unsigned int *)(GPIOI_BASE + 0x0008))    /* GPIOI_OSPEEDR 的地址为0x5000A008 */
#define GPIOI_PUPDR              *((volatile unsigned int *)(GPIOI_BASE + 0x000C))        /* GPIOI_PUPDR的地址为0x5000A00C */
#define GPIOI_BSRR              *((volatile unsigned int *)(GPIOI_BASE + 0x0018))     /* GPIOI_BSRR的地址为 0x5000A018 */

#define GPIOF_MODER              *((volatile unsigned int *)(GPIOF_BASE + 0x0000))        /* GPIOF_MODER的地址为0x50007000 */
#define GPIOF_OTYPER              *((volatile unsigned int *)(GPIOF_BASE + 0x0004))    /* GPIOF_OTYPER的地址为0x50007004 */
#define GPIOF_OSPEEDR              *((volatile unsigned int *)(GPIOF_BASE + 0x0008))    /* GPIOF_OSPEEDR的地址为0x50007008 */
#define GPIOF_PUPDR              *((volatile unsigned int *)(GPIOF_BASE + 0x000C))        /* GPIOF_PUPDR 的地址为 0x5000700C */
#define GPIOF_BSRR              *((volatile unsigned int *)(GPIOF_BASE + 0x0018))     /* GPIOF_BSRR 地址为0x50007018 */

#define RCC_MC_AHB5ENSETR        *((volatile unsigned int *)(RCC_BASE + 0X290))            /* RCC_MC_AHB5ENSETR寄存器的地址为0x50000290 */


void led0_switch(unsigned char state)
{
    if(state == OFF) 
    {
        GPIOI_BSRR |= ((unsigned int)1 << 0);     /* GPIOI_0输出高电平 */
    } else if(state == ON) 
    {
        GPIOI_BSRR |= ((unsigned int)1 << 16);     /* GPIOI_0输出低电平 */
    }
}

/**
 * @brief     LED1开关函数
 * @param     无
 * @retval    无
 */ 
void led1_switch(unsigned char state)
{
    if(state == OFF) 
    {
        GPIOF_BSRR |= ((unsigned int)1 << 3);     /* GPIOF_3输出高电平 */
    } else if(state == ON) 
    {
        GPIOF_BSRR |= ((unsigned int)1 << 19);     /* GPIOF_3输出低电平 */
    }
}

/**
 * @brief       主函数
 * @param       无
 * @retval      无
 */
int main(void)
{
    uint8_t t;
    uint8_t len;
    uint16_t times = 0; 

//    HAL_Init();        /* 初始化HAL库     */
    extern uint8_t g_rx_buffer;

    /* 初始化M4内核时钟 */
//    sys_stm32_clock_init(34, 2, 2, 17, 6826);
    clock_init(34, 2, 2, 17, 6826);

    usart_init(115200);     /* 串口初始化为115200 */
    led_init();             /* 初始化LED      */
    beep_init();            /* 初始化蜂鸣器 */

    printf("\r\n SystemCoreClockFreq: %d\r\n",HAL_RCC_GetSystemCoreClockFreq());
 //   wwdg_init(0x7F, 0x5F, WWDG_PRESCALER_16); /* 计数器值为7f,窗口寄存器为5f,分频数为16 */

    printf("\r\n UART8->BRR: 0x%X\r\n",UART8->BRR);
    printf("\r\n UART4->BRR: %d\r\n",UART4->BRR);
    printf("\r\n RCC->UART78CKSELR: 0x%X\r\n",RCC->UART78CKSELR);
    
    printf("\r\n");
    printf(" RCC->PLL1CR: 0x%X\r\n",RCC->PLL1CR);
    printf(" RCC->PLL1CFGR1: 0x%X\r\n",RCC->PLL1CFGR1);
    printf(" RCC->PLL1CFGR2: 0x%X\r\n",RCC->PLL1CFGR2);
    printf(" RCC->PLL1FRACR: 0x%X\r\n",RCC->PLL1FRACR);
    printf("\r\n");
    printf(" RCC->PLL2CR: 0x%X\r\n",RCC->PLL2CR);
    printf(" RCC->PLL2CFGR1: 0x%X\r\n",RCC->PLL2CFGR1);
    printf(" RCC->PLL2CFGR2: 0x%X\r\n",RCC->PLL2CFGR2);
    printf(" RCC->PLL2FRACR: 0x%X\r\n",RCC->PLL2FRACR);
    printf("\r\n");
    printf(" RCC->PLL3CR: 0x%X\r\n",RCC->PLL3CR);
    printf(" RCC->PLL3CFGR1: 0x%X\r\n",RCC->PLL3CFGR1);
    printf(" RCC->PLL3CFGR2: 0x%X\r\n",RCC->PLL3CFGR2);
    printf(" RCC->PLL3FRACR: 0x%X\r\n",RCC->PLL3FRACR);
    printf("\r\n");
    printf(" RCC->PLL4CR: 0x%X\r\n",RCC->PLL4CR);
    printf(" RCC->PLL4CFGR1: 0x%X\r\n",RCC->PLL4CFGR1);
    printf(" RCC->PLL4CFGR2: 0x%X\r\n",RCC->PLL4CFGR2);
    printf(" RCC->PLL4FRACR: 0x%X\r\n",RCC->PLL4FRACR);
    
    while(1)
    {
        led0_switch(ON);    /* LED0开 */
        led1_switch(OFF);    /* LED0关 */
    //    ledz_switch(ON);
        D_elay(500);                /* 延时一定时间 */
        led0_switch(OFF);    /* LED0关 */
        led1_switch(ON);        /* LED0开 */
   //     ledz_switch(OFF);
        D_elay(500);                /* 延时一定时间 */
    }
}
