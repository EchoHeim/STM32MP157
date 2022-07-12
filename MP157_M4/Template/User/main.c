#include "gpio.h"
#include "usart.h"

void BSP_Init(void)
{
    led_init();    /* 初始化LED  */
    key_init();     /* 初始化KEY  */
    usart_init(115200);  //初始化串口;
}

int main(void)
{
    HAL_Init();    /* 初始化HAL库 */
//    HAL_DeInit();
    
    /* 初始化M4内核时钟 */
    if(IS_ENGINEERING_BOOT_MODE())
    {
        sys_stm32_clock_init(34, 2, 2, 17, 6826);
    }
    
    BSP_Init();     // 板级驱动初始化;

    uint8_t key_num;
    uint8_t len,t;
    
    while(1)
    {
        key_num = key_scan(0);
        
        if(key_num == 1)
        {
            printf("\r\n The key1 is pressed! \r\n");
            LED0_TOGGLE();
        }
        else if(key_num == 2)
        {
            printf("\r\n The key2 is pressed! \r\n");
            LED1_TOGGLE();
        }
        if (g_usart_rx_sta & 0x8000)        /* 接收到了数据 */
        {
            len = g_usart_rx_sta & 0x3fff;  /* 得到此次接收到的数据长度 */
            
            if(g_usart_rx_buf[0] == '1')
            {
                printf("\r\nTurn on the led! \r\n");
                LED0(1);     /* 打开LED */
                LED1(1);
            }
            else if(g_usart_rx_buf[0] == '0')
            {
                printf("\r\nTurn off the led! \r\n");
                LED0(0);     /* 关闭LED */
                LED1(0);
            }
            
            printf("\r\n您发送的消息为:");
            for (t = 0; t < len; t++)
            {
                USART_UX->TDR = g_usart_rx_buf[t];
                while ((USART_UX->ISR & 0X40) == 0);  /* 等待发送结束 */
            }
            printf("\r\n\r\n"); /* 插入换行 */
            g_usart_rx_sta = 0;
        }
    }
}
