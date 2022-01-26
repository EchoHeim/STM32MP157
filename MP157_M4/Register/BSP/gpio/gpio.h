#ifndef __GPIO_H_
#define __GPIO_H_

#include "sys.h"
#include "delay.h"

/* IO引脚定义 */
#define LED0_GPIO_PORT                  GPIOI
#define LED0_GPIO_PIN                   GPIO_PIN_0

#define LED1_GPIO_PORT                  GPIOF
#define LED1_GPIO_PIN                   GPIO_PIN_3

#define BEEP_GPIO_PORT                  GPIOC
#define BEEP_GPIO_PIN                   GPIO_PIN_7

#define KEY1_GPIO_PORT                  GPIOG
#define KEY1_GPIO_PIN                   GPIO_PIN_3

#define KEY2_GPIO_PORT                  GPIOH
#define KEY2_GPIO_PIN                   GPIO_PIN_7


/* GPIO时钟使能 */
#define LED0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define LED1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)
#define BEEP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  


/* 端口定义 */
#define LED0(x)  do{ x ? \
                     HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_SET) : \
                     HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_RESET); \
                 }while(0)       /* LED0 = RED */

#define LED1(x)  do{ x ? \
                     HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : \
                     HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
                 }while(0)       /* LED1 = GREEN */

#define BEEP(x)  do{ x ? \
                     HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET) : \
                     HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET); \
                 }while(0)

#define KEY1     HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* 读取KEY1引脚 */
#define KEY2     HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)     /* 读取KEY2引脚 */


/* IO取反定义 */
#define LED0_TOGGLE()  do{ HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)     /* LED0 = !LED0 */
#define LED1_TOGGLE()  do{ HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)     /* LED1 = !LED1 */
#define BEEP_TOGGLE()  do{ HAL_GPIO_TogglePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN); }while(0)     /* BEEP = !BEEP */


void led_init(void);    /* 初始化 LED */
void beep_init(void);   //BEEP初始化函数
void key_init(void);           /* 按键初始化函数 */
uint8_t key_scan(uint8_t mode);/* 按键扫描函数 */


#define KEY1_PRES    1      /* KEY1按下 */
#define KEY2_PRES    2      /* KEY2按下 */

#endif
