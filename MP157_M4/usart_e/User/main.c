#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./SYSTEM/usart/usart.h"

/**
 * @brief       主函数
 * @param       无
 * @retval      无
 */
int main(void)
{
    HAL_Init();        /* 初始化HAL库     */
    extern uint8_t g_rx_buffer;
    /* 初始化M4内核时钟 */
    if(IS_ENGINEERING_BOOT_MODE())
    {
        sys_stm32_clock_init(34, 2, 2, 17, 6826);
    }
    usart_init(115200);     /* 串口初始化为115200 */
    led_init();                /* 初始化LED      */
    printf("\r\n正点原子 STM32MP1 开发板 串口实验\r\n");
     HAL_UART_Receive_IT(&g_uart4_handle,&g_rx_buffer,1);/* 以中断方式接收函数 */
    while(1) 
    {
        printf("请输入数据,以回车键结束\r\n");
         HAL_Delay(500);
    }
}
