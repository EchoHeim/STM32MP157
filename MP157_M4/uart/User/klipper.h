#ifndef __KLIPPER_H
#define __KLIPPER_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"

//#define GPIO_Rx GPIO('E', 0)
//#define GPIO_Tx GPIO('E', 1)

#define GPIO_Rx GPIO('B', 2)
#define GPIO_Tx GPIO('G', 11)


#define DIV_ROUND_CLOSEST(x, divisor)({                 \
            typeof(divisor) __divisor = divisor;        \
            (((x) + ((__divisor) / 2)) / (__divisor));  \
        })

extern GPIO_TypeDef * const digital_regs[];

#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3

void clock_setup(void);

void enable_pclock(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);
void gpio_peripheral_c(uint32_t gpio, uint32_t Mode,uint32_t Pull, uint32_t Speed, uint32_t AF_Mode);

uint32_t get_pclock_frequency(uint32_t periph_base);
void clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv);

#endif

