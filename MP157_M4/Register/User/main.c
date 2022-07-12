#include "gpio.h"
#include "usart.h"
#include "wwdg.h"

#include "stm32mp1xx.h"

// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
// periph_base determines in which bitfield at wich position to set a bit
    // E.g. D2_AHB1PERIPH_BASE is the adress offset of the given bitfield
    if (periph_base < APB2_PERIPH_BASE) {
        if(periph_base <= LPTIM1_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == WWDG1_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1C); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= SPI3_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == SPDIFRX_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1A); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= UART5_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= I2C5_BASE) {
            uint32_t pos = ((periph_base + 0x3000) - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == CEC_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1B); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == DAC1_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1D); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= UART8_BASE) {
            uint32_t pos = ((periph_base) - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
    } else if (periph_base < AHB2_PERIPH_BASE) {
        if(periph_base <= TIM8_BASE) {
            uint32_t pos = (periph_base - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base == USART6_BASE) {
            RCC->MC_APB2ENSETR |= (1<<0x0D); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base <= SPI4_BASE) {
            uint32_t pos = ((periph_base + 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base <= TIM17_BASE) {
            uint32_t pos = ((periph_base - 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base == SPI5_BASE) {
            RCC->MC_APB2ENSETR |= (1<<0x0A); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
    } else if (periph_base < AHB3_PERIPH_BASE) {
        if(periph_base <= DMAMUX1_BASE) {
            uint32_t pos = (periph_base - DMA1_BASE) / 0x1000;
            RCC->MC_AHB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_AHB2ENSETR;
        } if(periph_base <= ADC12_COMMON_BASE) {
            RCC->MC_AHB2ENSETR |= RCC_MC_AHB2ENSETR_ADC12EN;
            RCC->MC_AHB2ENSETR;
        }
    } else if (periph_base < AHB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - AHB3_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB3ENSETR |= (1<<pos);
        RCC->MC_AHB3ENSETR;
    } else if (periph_base < APB3_PERIPH_BASE) {
        uint32_t pos = ((periph_base - 0x2000) - AHB4_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB4ENSETR |= (1<<pos);
        RCC->MC_AHB4ENSETR;
    } else if (periph_base < APB_DEBUG_PERIPH_BASE) {
        uint32_t pos = (periph_base - APB3_PERIPH_BASE) / 0x1000;
        RCC->MC_APB3ENSETR |= (1<<pos);
        RCC->MC_APB3ENSETR;
    } else if (periph_base < GPV_PERIPH_BASE) {
        if(periph_base == GPIOZ_BASE) {
            RCC->MC_AHB5ENSETR |= (1<<0x00); // we assume it is not in APB1HENR
            RCC->MC_AHB5ENSETR;
        }
    } else if (periph_base < APB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - AHB6_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB6ENSETR |= (1<<pos);
        RCC->MC_AHB6ENSETR;
    } else if (periph_base < APB5_PERIPH_BASE) {
        uint32_t pos = (periph_base - APB4_PERIPH_BASE) / 0x1000;
        RCC->MC_APB4ENSETR |= (1<<pos);
        RCC->MC_APB4ENSETR;
    } else {
        uint32_t pos = (periph_base - APB5_PERIPH_BASE) / 0x1000;
        RCC->MC_APB5ENSETR |= (1<<pos);
        RCC->MC_APB5ENSETR;
    }
}

#define STM_OSPEED 0x2 // ~85Mhz at 50pF

GPIO_TypeDef * const digital_regs[] = {
    ['A' - 'A'] = GPIOA, GPIOB, GPIOC,
#ifdef GPIOD
    ['D' - 'A'] = GPIOD,
#endif
#ifdef GPIOE
    ['E' - 'A'] = GPIOE,
#endif
#ifdef GPIOF
    ['F' - 'A'] = GPIOF,
#endif
#ifdef GPIOG
    ['G' - 'A'] = GPIOG,
#endif
#ifdef GPIOH
    ['H' - 'A'] = GPIOH,
#endif
#ifdef GPIOI
    ['I' - 'A'] = GPIOI,
#endif
#ifdef GPIOZ
    ['Z' - 'A'] = GPIOZ,
#endif
};

void gpio_peripheral_K(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    enable_pclock((uint32_t)regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf, od = mode >> 8;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (STM_OSPEED << m_shift);
}

void gpio_peripheral(uint32_t gpio, uint32_t Mode,uint32_t Pull, uint32_t Speed, uint32_t AF_Mode)
{
    GPIO_TypeDef *regs =  digital_regs[GPIO2PORT(gpio)];
    uint32_t temp;

    enable_pclock((uint32_t)regs);      // Enable GPIO clock

    uint32_t pos = gpio % 16;

    /* Configure Alternate function mapped with the current IO */
    temp = regs->AFR[pos >> 3];
    temp &= ~((uint32_t)0xF << ((uint32_t)(pos & (uint32_t)0x07) * 4)) ;
    temp |= ((uint32_t)(AF_Mode) << (((uint32_t)pos & (uint32_t)0x07) * 4));
    regs->AFR[pos >> 3] = temp;

    /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
    temp = regs->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (pos * 2));
    temp |= ((Mode & ((uint32_t)0x00000003)) << (pos * 2));
    regs->MODER = temp;

    /* Configure the IO Speed */
    temp = regs->OSPEEDR;
    temp &= ~(GPIO_OSPEEDR_OSPEEDR0 << (pos * 2));
    temp |= (Speed << (pos * 2));
    regs->OSPEEDR = temp;

    /* Configure the IO Output Type */
    temp = regs->OTYPER;
    temp &= ~(GPIO_OTYPER_OT0 << pos) ;
    temp |= (((Mode & ((uint32_t)0x00000010)) >> 4) << pos);
    regs->OTYPER = temp;

    /* Activate the Pull-up or Pull down resistor for the current IO */
    temp = regs->PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (pos * 2));
    temp |= ((Pull) << (pos * 2));
    regs->PUPDR = temp;
}

int main(void)
{
 //   HAL_Init();    /* 初始化HAL库 */
//    HAL_DeInit();

//    SystemInit();
    /* 初始化M4内核时钟 */
//    if(IS_ENGINEERING_BOOT_MODE())
    {
#if 1
        /* Enable access to RTC and backup registers */
        
        SCB->AIRCR =  0x05FA0300;
        SystemCoreClock = 0x0C7516C0;
        
        
        SET_BIT(PWR->CR1, PWR_CR1_DBP);
        MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, (uint32_t)(RCC_BDCR_LSEDRV_1));      // 允许修改LSE;
        
        /* Disable HSEON before configuring the HSE --------------*/
        WRITE_REG(RCC->OCENCLRR, RCC_OCENCLRR_HSEON);
        
        /* Clear remaining bits */
        WRITE_REG(RCC->OCENCLRR, (RCC_OCENCLRR_DIGBYP | RCC_OCENSETR_HSEBYP));
        
        /* Enable oscillator */
        SET_BIT(RCC->OCENSETR, RCC_OCENSETR_HSEON);
        
        /* Enable the Internal Low Speed oscillator (LSI). */
        SET_BIT(RCC->RDLSICR, RCC_RDLSICR_LSION);
        
        
        WRITE_REG(RCC->PLL1CR, (0x73));
        WRITE_REG(RCC->PLL1CFGR1, (0x20063));
        WRITE_REG(RCC->PLL1CFGR2, (0x0));
        WRITE_REG(RCC->PLL1FRACR, (0x10000));
        
        WRITE_REG(RCC->PLL2CR, (0x73));
        WRITE_REG(RCC->PLL2CFGR1, (0x20041));
        WRITE_REG(RCC->PLL2CFGR2, (0x1));
        WRITE_REG(RCC->PLL2FRACR, (0x1A000));
        
        WRITE_REG(RCC->PLL3CR, (0x73));
        WRITE_REG(RCC->PLL3CFGR1, (0x1010021));
        WRITE_REG(RCC->PLL3CFGR2, (0x241001));
        WRITE_REG(RCC->PLL3FRACR, (0x1D550));
        
        WRITE_REG(RCC->PLL4CR, (0x73));
        WRITE_REG(RCC->PLL4CFGR1, (0x30062));
        WRITE_REG(RCC->PLL4CFGR2, (0x70705));
        WRITE_REG(RCC->PLL4FRACR, (0x10000));

#else
        sys_stm32_clock_init(34, 2, 2, 17, 6826);
#endif
    }
    
    gpio_peripheral_K(GPIO('F',3), GPIO_OUTPUT, 0);
    GPIOF->BSRR |= ((unsigned int)1 << 3);     /* GPIOZ_0输出高电平 */
    
//    led_init();    /* 初始化LED  */
//    key_init();     /* 初始化KEY  */
    usart_init(115200);  //初始化串口;

    uint8_t key_num;
    uint8_t len,t;
    
    uint32_t que = HAL_RCC_GetSystemCoreClockFreq();
    printf("\r\n SystemCoreClockFreq：0x%X\r\n",que);
 //   wwdg_init(0x7F, 0x5F, WWDG_PRESCALER_16); /* 计数器值为7f,窗口寄存器为5f,分频数为16 */
    que = UART8->BRR;
    printf("\r\n SystemCoreClockFreq：0x%X\r\n",que);
    printf("\r\n RCC->UART78CKSELR: 0x%X\r\n",RCC->UART78CKSELR);
    
    printf("\r\n RCC->PLL1CR: 0x%X\r\n",RCC->PLL1CR);
    printf("\r\n RCC->PLL1CFGR1: 0x%X\r\n",RCC->PLL1CFGR1);
    printf("\r\n RCC->PLL1CFGR2: 0x%X\r\n",RCC->PLL1CFGR2);
    printf("\r\n RCC->PLL1FRACR: 0x%X\r\n",RCC->PLL1FRACR);
    
    printf("\r\n RCC->PLL2CR: 0x%X\r\n",RCC->PLL2CR);
    printf("\r\n RCC->PLL2CFGR1: 0x%X\r\n",RCC->PLL2CFGR1);
    printf("\r\n RCC->PLL2CFGR2: 0x%X\r\n",RCC->PLL2CFGR2);
    printf("\r\n RCC->PLL2FRACR: 0x%X\r\n",RCC->PLL2FRACR);
    
    printf("\r\n RCC->PLL3CR: 0x%X\r\n",RCC->PLL3CR);
    printf("\r\n RCC->PLL3CFGR1: 0x%X\r\n",RCC->PLL3CFGR1);
    printf("\r\n RCC->PLL3CFGR2: 0x%X\r\n",RCC->PLL3CFGR2);
    printf("\r\n RCC->PLL3FRACR: 0x%X\r\n",RCC->PLL3FRACR);
    
    printf("\r\n RCC->PLL4CR: 0x%X\r\n",RCC->PLL4CR);
    printf("\r\n RCC->PLL4CFGR1: 0x%X\r\n",RCC->PLL4CFGR1);
    printf("\r\n RCC->PLL4CFGR2: 0x%X\r\n",RCC->PLL4CFGR2);
    printf("\r\n RCC->PLL4FRACR: 0x%X\r\n",RCC->PLL4FRACR);
    
    
 //   while(1)
    {
        for(int i =0;i<2900;i++)
            for(int j =0;j<2900;j++);
        GPIOF->BSRR |= ((unsigned int)1 << 3); 
        
        for(int i =0;i<1900;i++)
            for(int j =0;j<2900;j++);
        GPIOF->BSRR |= ((unsigned int)1 << 19); 
    }
    {
        key_num = key_scan(0);
        
        if(key_num == 1)
        {
            printf("\r\n The key1 is pressed! \r\n");
            GPIOZ->BSRR |= ((unsigned int)1 << 0);     /* GPIOZ_0输出高电平 */
            LED0_TOGGLE();
        }
        else if(key_num == 2)
        {
            printf("\r\n The key2 is pressed! \r\n");
            GPIOZ->BSRR |= ((unsigned int)1 << 16);     /* GPIOZ_0输出高电平 */
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
                USARTx->TDR = g_usart_rx_buf[t];
                while ((USARTx->ISR & 0X40) == 0);  /* 等待发送结束 */
            }
            printf("\r\n\r\n"); /* 插入换行 */
            g_usart_rx_sta = 0;
        }
    }
}
