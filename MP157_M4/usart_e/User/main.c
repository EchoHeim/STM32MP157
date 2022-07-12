#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./SYSTEM/usart/usart.h"

/**
 * @brief       ������
 * @param       ��
 * @retval      ��
 */
int main(void)
{
    HAL_Init();        /* ��ʼ��HAL��     */
    extern uint8_t g_rx_buffer;
    /* ��ʼ��M4�ں�ʱ�� */
    if(IS_ENGINEERING_BOOT_MODE())
    {
        sys_stm32_clock_init(34, 2, 2, 17, 6826);
    }
    usart_init(115200);     /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                /* ��ʼ��LED      */
    printf("\r\n����ԭ�� STM32MP1 ������ ����ʵ��\r\n");
     HAL_UART_Receive_IT(&g_uart4_handle,&g_rx_buffer,1);/* ���жϷ�ʽ���պ��� */
    while(1) 
    {
        printf("����������,�Իس�������\r\n");
         HAL_Delay(500);
    }
}
