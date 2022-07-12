#include "gpio.h"
#include "usart.h"

void BSP_Init(void)
{
    led_init();    /* ��ʼ��LED  */
    key_init();     /* ��ʼ��KEY  */
    usart_init(115200);  //��ʼ������;
}

int main(void)
{
    HAL_Init();    /* ��ʼ��HAL�� */
//    HAL_DeInit();
    
    /* ��ʼ��M4�ں�ʱ�� */
    if(IS_ENGINEERING_BOOT_MODE())
    {
        sys_stm32_clock_init(34, 2, 2, 17, 6826);
    }
    
    BSP_Init();     // �弶������ʼ��;

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
        if (g_usart_rx_sta & 0x8000)        /* ���յ������� */
        {
            len = g_usart_rx_sta & 0x3fff;  /* �õ��˴ν��յ������ݳ��� */
            
            if(g_usart_rx_buf[0] == '1')
            {
                printf("\r\nTurn on the led! \r\n");
                LED0(1);     /* ��LED */
                LED1(1);
            }
            else if(g_usart_rx_buf[0] == '0')
            {
                printf("\r\nTurn off the led! \r\n");
                LED0(0);     /* �ر�LED */
                LED1(0);
            }
            
            printf("\r\n�����͵���ϢΪ:");
            for (t = 0; t < len; t++)
            {
                USART_UX->TDR = g_usart_rx_buf[t];
                while ((USART_UX->ISR & 0X40) == 0);  /* �ȴ����ͽ��� */
            }
            printf("\r\n\r\n"); /* ���뻻�� */
            g_usart_rx_sta = 0;
        }
    }
}
