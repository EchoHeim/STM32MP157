#ifndef __GPIO_H_
#define __GPIO_H_

#include "sys.h"
#include "delay.h"

/* IO���Ŷ��� */
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


/* GPIOʱ��ʹ�� */
#define LED0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)
#define LED1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)
#define BEEP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  


/* �˿ڶ��� */
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

#define KEY1     HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* ��ȡKEY1���� */
#define KEY2     HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)     /* ��ȡKEY2���� */


/* IOȡ������ */
#define LED0_TOGGLE()  do{ HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)     /* LED0 = !LED0 */
#define LED1_TOGGLE()  do{ HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)     /* LED1 = !LED1 */
#define BEEP_TOGGLE()  do{ HAL_GPIO_TogglePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN); }while(0)     /* BEEP = !BEEP */


void led_init(void);    /* ��ʼ�� LED */
void beep_init(void);   //BEEP��ʼ������
void key_init(void);           /* ������ʼ������ */
uint8_t key_scan(uint8_t mode);/* ����ɨ�躯�� */


#define KEY1_PRES    1      /* KEY1���� */
#define KEY2_PRES    2      /* KEY2���� */

#endif
