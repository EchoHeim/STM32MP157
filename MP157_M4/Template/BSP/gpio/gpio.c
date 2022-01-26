#include "gpio.h"

/**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    LED0_GPIO_CLK_ENABLE(); /* LED0时钟使能 */
    LED1_GPIO_CLK_ENABLE(); /* LED1时钟使能 */

    gpio_init_struct.Pin = LED0_GPIO_PIN;                   /* LED0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速 */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* 初始化LED0引脚 */

    gpio_init_struct.Pin = LED1_GPIO_PIN;                   /* LED1引脚 */
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);       /* 初始化LED1引脚 */
    
    LED0(0);    /* 关闭 LED0 */
    LED1(0);    /* 关闭 LED1 */
}


/**
 * @brief       初始化BEEP相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void beep_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    BEEP_GPIO_CLK_ENABLE();                             /* BEEP时钟使能 */

    gpio_init_struct.Pin = BEEP_GPIO_PIN;               /* 蜂鸣器引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    HAL_GPIO_Init(BEEP_GPIO_PORT, &gpio_init_struct);   /* 初始化蜂鸣器引脚 */

    BEEP(0);                                            /* 关闭蜂鸣器 */
}

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    KEY1_GPIO_CLK_ENABLE(); /* KEY1时钟使能 */
    KEY2_GPIO_CLK_ENABLE(); /* KEY2时钟使能 */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                     /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                  /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;       /* 高速 */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);         /* KEY0引脚初始化 */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;                     /* KEY1引脚 */
    gpio_init_struct.Pull = GPIO_PULLUP;                      /* 上拉 */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);         /* KEY1引脚初始化 */
}

/**
 * @brief       按键扫描函数
 * @param       mode:可取 0 / 1, 具体含义如下:
 *   @arg       0, 不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开按键以后, 再次按下才会返回其他键值)
 *   @arg       1, 支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY1_PRES, 1, KEY1按下
 *              KEY2_PRES, 2, KEY2按下
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* 支持连按 */
        /* 按键松开标志为1, 且有任意一个按键按下了 */
    if (key_up && (KEY1 == 0 || KEY2 == 0 ))   
    {
        //delay(2);               /* 去抖动，后面会换成高精度延时函数！ */
        HAL_Delay(10);
        key_up = 0;

        if (KEY1 == 0)
            keyval = KEY1_PRES;

        if (KEY2 == 0)  
            keyval = KEY2_PRES;

    }
    else if (KEY1 == 1 && KEY2 == 1 )  /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;  /* 返回键值 */
}
