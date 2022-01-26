#include "gpio.h"

/**
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    LED0_GPIO_CLK_ENABLE(); /* LED0ʱ��ʹ�� */
    LED1_GPIO_CLK_ENABLE(); /* LED1ʱ��ʹ�� */

    gpio_init_struct.Pin = LED0_GPIO_PIN;                   /* LED0���� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* ���� */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* ��ʼ��LED0���� */

    gpio_init_struct.Pin = LED1_GPIO_PIN;                   /* LED1���� */
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);       /* ��ʼ��LED1���� */
    
    LED0(0);    /* �ر� LED0 */
    LED1(0);    /* �ر� LED1 */
}


/**
 * @brief       ��ʼ��BEEP���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void beep_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    BEEP_GPIO_CLK_ENABLE();                             /* BEEPʱ��ʹ�� */

    gpio_init_struct.Pin = BEEP_GPIO_PIN;               /* ���������� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ���� */
    HAL_GPIO_Init(BEEP_GPIO_PORT, &gpio_init_struct);   /* ��ʼ������������ */

    BEEP(0);                                            /* �رշ����� */
}

/**
 * @brief       ������ʼ������
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    KEY1_GPIO_CLK_ENABLE(); /* KEY1ʱ��ʹ�� */
    KEY2_GPIO_CLK_ENABLE(); /* KEY2ʱ��ʹ�� */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                     /* KEY0���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                  /* ���� */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       /* ���� */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);         /* KEY0���ų�ʼ�� */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;                     /* KEY1���� */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* ���� */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);         /* KEY1���ų�ʼ�� */
}

/**
 * @brief       ����ɨ�躯��
 * @param       mode:��ȡ 0 / 1, ���庬������:
 *   @arg       0, ��֧��������(���������²���ʱ, ֻ�е�һ�ε��û᷵�ؼ�ֵ,
 *                  �����ɿ������Ժ�, �ٴΰ��²Ż᷵��������ֵ)
 *   @arg       1, ֧��������(���������²���ʱ, ÿ�ε��øú������᷵�ؼ�ֵ)
 * @retval      ��ֵ, ��������:
 *              KEY1_PRES, 1, KEY1����
 *              KEY2_PRES, 2, KEY2����
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* �������ɿ���־ */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* ֧������ */
        /* �����ɿ���־Ϊ1, ��������һ������������ */
    if (key_up && (KEY1 == 0 || KEY2 == 0 ))   
    {
        //delay(2);               /* ȥ����������ỻ�ɸ߾�����ʱ������ */
        HAL_Delay(10);
        key_up = 0;

        if (KEY1 == 0)
            keyval = KEY1_PRES;

        if (KEY2 == 0)  
            keyval = KEY2_PRES;

    }
    else if (KEY1 == 1 && KEY2 == 1 )  /* û���κΰ�������, ��ǰ����ɿ� */
    {
        key_up = 1;
    }

    return keyval;  /* ���ؼ�ֵ */
}
