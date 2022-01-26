/**
 ****************************************************************************************************
 * @file        sys.c
 * @author      正点原子Linux团队(ALIENTEK)
 * @version     V1.1
 * @date        2020-05-04
 * @brief       系统初始化代码(包括时钟配置/中断管理/GPIO设置等)
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32MP1开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200504
 ****************************************************************************************************
 */
 
#include "./SYSTEM/sys/sys.h"

/**
 * @brief       M4主频时钟设置函数，也就是设置PLL3
 * @param       plln: PLL3倍频系数(PLL倍频), 取值范围: 25~200.
 * @param       pllm: PLL3预分频系数(进PLL之前的分频), 取值范围: 1~64.
 * @param       pllp: PLL3的p分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 1~128.(且必须是2的倍数)
 * @param       pllq: PLL3的q分频系数(PLL之后的分频), 取值范围: 1~128.
 * @note
 *
 *              Fvco: VCO频率
 *              Fsys: 系统时钟频率, 也是PLL1的p分频输出时钟频率
 *              Fq:   PLL1的q分频输出时钟频率
 *              Fs:   PLL输入时钟频率, 可以是HSI, CSI, HSE等.
 *              Fvco = (Fs / pllm) * (plln + (pllfracv / 8192))
 *              Fsys = Fvco / pllp;
 *              Fq   = Fvco / pllq;
 *
 *              外部晶振为24M的时候, 推荐值: plln = 34, pllm = 2, pllp = 2, pllq = 17, pllfracv = 6826.
 *              得到:Fvco = (24 / 2) * (34 + (6826 / 8192)) = 417.999MHz ≈ 418MHz
 *                   Fsys = Fvco / pllp = 418 / 2  = 209MHz
 *                   Fq   = Fvco / pllq = 418 / 17 = 24.588MHz
 *
 *              MP157默认需要配置的频率如下:
 *              CPU频率(mcu_ck) = MLHCLK = PLL3P / 1 = 209Mhz
 *              hclk = MLHCLK = 209Mhz
 *              AHB1/2/3/4 = hclk = 209Mhz
 *              APB1/2/3 = MLHCLK / 2 = 104.5Mhz
 * @retval      错误代码: 0, 成功; 1, 错误;
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv)
{
    RCC_OscInitTypeDef rcc_osc_init_handle;
    RCC_ClkInitTypeDef rcc_clk_init_handle;
    
    SET_BIT(PWR->CR1, PWR_CR1_DBP);    /* 使能访问backup区域     */
    
    MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, (uint32_t)(RCC_LSEDRIVE_MEDIUMHIGH));    /* 允许修改LSE    */
    
    /* 初始化CPU、AHB和APB等时钟 */
    
    /**
     * 使能HSE，将HSE(24MHz有源晶振)作为PLL时钟源
     */
//    rcc_osc_init_handle.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | \
//                                        RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_LSI| \
//                                        RCC_OSCILLATORTYPE_CSI;
//    
//    rcc_osc_init_handle.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI ;
//    
//    rcc_osc_init_handle.HSEState = RCC_HSE_ON;        /* 打开HSE */
//    rcc_osc_init_handle.LSEState = RCC_LSE_OFF;        /* 打开LSE */
//    rcc_osc_init_handle.HSIState = RCC_HSI_OFF;        /* 打开HSI */
//    rcc_osc_init_handle.CSIState = RCC_CSI_OFF;        /* 打开CSI */
//    rcc_osc_init_handle.LSIState = RCC_LSI_ON;        /* 打开HSI */
//    rcc_osc_init_handle.HSICalibrationValue = 16;
//    rcc_osc_init_handle.HSIDivValue = RCC_HSI_DIV1;
    
    /**
     * 配置PLL1，PLL1主要为MPU系统时钟，也就是A7的时钟
     * A7系统时钟计算公式为：Fref1_ck = 24M / PLLM = 24M / 3 = 8MHz
     *                       PLL1_vco = Fref1_ck * 2 *((DIVN + 1) + (FRACV / 8192))
     *                         PLL1_vco = 8MHz * 2 * (100 + (0/ 8192))     
     *                                = 1600MHz
     */
//    rcc_osc_init_handle.PLL.PLLState = RCC_PLL_ON;
//    rcc_osc_init_handle.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
//    rcc_osc_init_handle.PLL.PLLM = 3;
//    rcc_osc_init_handle.PLL.PLLN = 100;        /* 注意这里写的是加1以后的PLLN(DIVN)值     */
//    rcc_osc_init_handle.PLL.PLLP = 1;        /* PLL1P路时钟=PLL1_vco / 2 / PLLP = 800MHz     */
//    rcc_osc_init_handle.PLL.PLLQ = 1;        /* PLL1Q路时钟=PLL1_vco / 2 / PLLQ = 800MHz     */
//    rcc_osc_init_handle.PLL.PLLR = 1;        /* PLL1R路时钟=PLL1_vco / 2 / PLLR = 800MHz     */
//    rcc_osc_init_handle.PLL.PLLFRACV = 0;
//    rcc_osc_init_handle.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
//    rcc_osc_init_handle.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
//    rcc_osc_init_handle.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * 配置PLL2，PLL2主要为AXI、AHB5、AHB6、APB4、APB5、GPU等时钟
     * PLL2也分为三路出去：PLL2P、PLL2Q和PLL2R。这三路均从PLL2N配置出来
     *                    Fref2_ck = 24M / PLL1M = 24M / 3 = 8MHz
     *                  PLL2_vco = Fref2_ck * 2 *((DIVN + 1) + (FRACV / 8192))
     *                             = 8MHz * 2 * (66 + (5120 / 8192))
     *                             = 1066MHz
     */
//    rcc_osc_init_handle.PLL2.PLLState = RCC_PLL_ON;                /* 打开PLL2 */
//    rcc_osc_init_handle.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;    /* PLL2时钟源为HSE=24M */
//    rcc_osc_init_handle.PLL2.PLLM = 3;
//    rcc_osc_init_handle.PLL2.PLLN = 66;        /* 注意这里写的是加1以后的PLLN(DIVN)值     */
//    rcc_osc_init_handle.PLL2.PLLP = 2;        /* PLL2P路时钟=PLL2_vco / 2 / PLLP = 266.5MHz     */
//    rcc_osc_init_handle.PLL2.PLLQ = 1;        /* PLL2Q路时钟=PLL2_vco / 2 / PLLQ = 533MHz     */
//    rcc_osc_init_handle.PLL2.PLLR = 1;        /* PLL2R路时钟=PLL2_vco / 2 / PLLR = 533MHz     */
//    rcc_osc_init_handle.PLL2.PLLFRACV = 5120;
//    rcc_osc_init_handle.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
//    rcc_osc_init_handle.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
//    rcc_osc_init_handle.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * 配置PLL3，PLL3主要为MCU、MCU Systick、AHB1~AHB4、APB1~APB3外设和时钟、DFSDM时钟
     * PLL3也分为三路出去：PLL3P、PLL3Q和PLL3R。这三路均从PLL3N配置出来
     *                    Fref3_ck = 24M / PLL3M = 24M / 2 = 12MHz
     *                  PLL3_vco = Fref3_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 12MHz * (34 + (6826 / 8192))
     *                             = 417.999MHz
     *                             ≈ 418MHz
     */
//    rcc_osc_init_handle.PLL3.PLLState = RCC_PLL_ON;                /* 打开PLL3 */
//    rcc_osc_init_handle.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;    /* PLL3时钟源为HSE=24MHz */
//    rcc_osc_init_handle.PLL3.PLLM = pllm;    
//    rcc_osc_init_handle.PLL3.PLLN = plln;        /* 注意这里写的是加1以后的PLLN(DIVN)值     */
//    rcc_osc_init_handle.PLL3.PLLP = pllp;        /* PLL3P路时钟=PLL3_vco / PLLP = 208.9995MHz     */
//    rcc_osc_init_handle.PLL3.PLLQ = pllq;        /* PLL3Q路时钟=PLL3_vco / PLLQ = 24.5881MHz     */
//    rcc_osc_init_handle.PLL3.PLLR = 37;            /* PLL3R路时钟=PLL3_vco / PLLR = 11.2972MHz     */
//    rcc_osc_init_handle.PLL3.PLLRGE = RCC_PLL3IFRANGE_1; /* ref3_ck范围为8~16Mhz */
//    rcc_osc_init_handle.PLL3.PLLFRACV = pllfracv;
//    rcc_osc_init_handle.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
//    rcc_osc_init_handle.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
//    rcc_osc_init_handle.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * 配置PLL4，某些外设时钟源，比如LTDC、DSI等
     * PLL4也分为三路出去：PLL4P、PLL4Q和PLL4R。这三路均从PLL4N配置出来
     *                    Fref4_ck = 24M / PLL4M = 24M / 4 = 6MHz
     *                  PLL4_vco = Fref4_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 6MHz * (99 + (0 / 8192))
     *                             = 594MHz
     */
//    rcc_osc_init_handle.PLL4.PLLState = RCC_PLL_ON;                /* 打开PLL4 */
//    rcc_osc_init_handle.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;    /* PLL4时钟源为HSE=24MHz */
//    rcc_osc_init_handle.PLL4.PLLM = 4;
//    rcc_osc_init_handle.PLL4.PLLN = 99;            /* 注意这里写的是加1以后的PLLN(DIVN)值     */
//    rcc_osc_init_handle.PLL4.PLLP = 6;            /* PLL4P路时钟=PLL4_vco / PLLP = 99MHz     */
//    rcc_osc_init_handle.PLL4.PLLQ = 8;            /* PLL4Q路时钟=PLL4_vco / PLLQ = 74.25MHz     */
//    rcc_osc_init_handle.PLL4.PLLR = 8;            /* PLL4R路时钟=PLL4_vco / PLLR = 74.25MHz     */
//    rcc_osc_init_handle.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
//    rcc_osc_init_handle.PLL4.PLLFRACV = 0;
//    rcc_osc_init_handle.PLL4.PLLMODE = RCC_PLL_INTEGER;
//    rcc_osc_init_handle.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
//    rcc_osc_init_handle.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    WRITE_REG(RCC->OCENCLRR, RCC_OCENCLRR_HSEON);

    /* Wait till HSE is disabled */
 //   while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET);
    while( (RCC->OCRDYR & RCC_OCRDYR_HSERDY) == RCC_OCRDYR_HSERDY );

    /* Clear remaining bits */
    WRITE_REG(RCC->OCENCLRR, (RCC_OCENCLRR_DIGBYP | RCC_OCENSETR_HSEBYP));

  /* Enable HSE if needed ---------------------------------------*/

//    if (rcc_osc_init_handle.HSEState == RCC_HSE_BYPASS)
//    {
//      SET_BIT(RCC->OCENSETR, RCC_OCENSETR_HSEBYP);
//    }
//    else if (rcc_osc_init_handle.HSEState == RCC_HSE_BYPASS_DIG)
//    {
//      SET_BIT(RCC->OCENSETR, RCC_OCENCLRR_DIGBYP);
//      SET_BIT(RCC->OCENSETR, RCC_OCENSETR_HSEBYP);
//    }

    /* Enable oscillator */
    SET_BIT(RCC->OCENSETR, RCC_OCENSETR_HSEON);
    while( (RCC->OCRDYR & RCC_OCRDYR_HSERDY) != RCC_OCRDYR_HSERDY );

    SET_BIT(RCC->RDLSICR, RCC_RDLSICR_LSION);

    /* Wait till LSI is ready */
    while((RCC->RDLSICR & RCC_RDLSICR_LSIRDY)  !=  RCC_RDLSICR_LSIRDY );
    
    //---------- Configure PLL1 -------------------------------------------
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1_DIVP | RCC_PLL1_DIVQ | RCC_PLL1_DIVR );
    
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1CR_PLLON);       /* Disable the main PLL. */
 
    /* Wait till PLL is ready */
    while((RCC->PLL1CR & RCC_PLL1CR_PLL1RDY) ==  RCC_PLL1CR_PLL1RDY );

    /* Configure PLL1 and PLL2 clock source */
    MODIFY_REG( RCC->RCK12SELR, RCC_RCK12SELR_PLL12SRC, RCC_RCK12SELR_PLL12SRC_1 );

    /* Wait till PLL SOURCE is ready */
    while( (RCC->RCK12SELR & RCC_RCK12SELR_PLL12SRCRDY) != RCC_RCK12SELR_PLL12SRCRDY );

    /* Configure the PLL1 multiplication and division factors. */        
    MODIFY_REG( RCC->PLL1CFGR1, (RCC_PLL1CFGR1_DIVN | RCC_PLL1CFGR1_DIVM1) , ( 99U | (2U << 16U )));
    MODIFY_REG( RCC->PLL1CFGR2, (RCC_PLL1CFGR2_DIVP | RCC_PLL1CFGR2_DIVQ | RCC_PLL1CFGR2_DIVR), ( 0U | ( (0U) <<8U ) | ( (0U) <<16U) ));

    /* Configure the Fractional Divider */
    CLEAR_BIT(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACLE); /*Set FRACLE to '0' */

    /* Configure PLL  PLL1FRACV  in fractional mode*/
    MODIFY_REG(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACV, (uint32_t)0 << RCC_PLL1FRACR_FRACV_Pos);

    //__HAL_RCC_PLL1FRACV_ENABLE(); 
    SET_BIT(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACLE);/* Set FRACLE to 1 */

    /* Configure the Spread Control */
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1CR_SSCG_CTRL);

    /* Enable the PLL1. */
    SET_BIT(RCC->PLL1CR, RCC_PLL1CR_PLLON );

    /* Wait till PLL is ready */
  //  while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL1RDY) == RESET);
    while((RCC->PLL1CR & RCC_PLL1CR_PLL1RDY)  !=  RCC_PLL1CR_PLL1RDY );

    /* Enable post-dividers */
    SET_BIT(RCC->PLL1CR, RCC_PLL1_DIVP | RCC_PLL1_DIVQ | RCC_PLL1_DIVR );
    
    //-------- Configure PLL2 --------------------------------------
        CLEAR_BIT(RCC->PLL2CR, RCC_PLL2_DIVP | RCC_PLL2_DIVQ | RCC_PLL2_DIVR );     /*Disable the post-dividers*/
        CLEAR_BIT(RCC->PLL2CR, RCC_PLL2CR_PLLON);   /* Disable the main PLL. */

        /* Wait till PLL is ready */
        while((RCC->PLL2CR & RCC_PLL2CR_PLL2RDY) ==  RCC_PLL2CR_PLL2RDY );
        
         /* Configure PLL1 and PLL2 clock source */
        MODIFY_REG( RCC->RCK12SELR, RCC_RCK12SELR_PLL12SRC, RCC_RCK12SELR_PLL12SRC_1 );
        
        /* Configure the PLL2 multiplication and division factors. */
         MODIFY_REG( RCC->PLL2CFGR1, (RCC_PLL2CFGR1_DIVN | RCC_PLL2CFGR1_DIVM2) , ( (65U) | ( (2U) << 16U) ) );
         MODIFY_REG( RCC->PLL2CFGR2, (RCC_PLL2CFGR2_DIVP | RCC_PLL2CFGR2_DIVQ | RCC_PLL2CFGR2_DIVR), ( (1U) | ( (0U) <<8U ) | ( (0U) <<16U) )); \

        /* Configure the Fractional Divider */
        CLEAR_BIT(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACLE); //Set FRACLE to ‘0’
        
        /* In integer or clock spreading mode the application shall ensure that a 0 is loaded into the SDM */        
        /* Configure PLL  PLL2FRACV  in fractional mode*/
        MODIFY_REG(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACV, (uint32_t)(5120) << RCC_PLL2FRACR_FRACV_Pos);
        
       // __HAL_RCC_PLL2FRACV_ENABLE(); 
        SET_BIT(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACLE); //Set FRACLE to ‘1’

        CLEAR_BIT(RCC->PLL2CR, RCC_PLL2CR_SSCG_CTRL);

        /* Enable the PLL2. */
        SET_BIT(RCC->PLL2CR, RCC_PLL2CR_PLLON );

        /* Wait till PLL is ready */
        while((RCC->PLL2CR & RCC_PLL2CR_PLL2RDY) !=  RCC_PLL2CR_PLL2RDY );

        /*Enable the post-dividers*/
        SET_BIT(RCC->PLL2CR, RCC_PLL2_DIVP | RCC_PLL2_DIVQ | RCC_PLL2_DIVR );
        
        //------- Configure PLL3 ---------------------------
        
          CLEAR_BIT(RCC->PLL3CR, RCC_PLL3_DIVP | RCC_PLL3_DIVQ | RCC_PLL3_DIVR );/*Disable the post-dividers*/
          CLEAR_BIT(RCC->PLL3CR, RCC_PLL3CR_PLLON); /* Disable the main PLL. */

        /* Wait till PLL is ready */
        while((RCC->PLL3CR & RCC_PLL3CR_PLL3RDY) ==  RCC_PLL3CR_PLL3RDY );

        /* Configure PLL3 clock source */
        MODIFY_REG( RCC->RCK3SELR, RCC_RCK3SELR_PLL3SRC, RCC_RCK3SELR_PLL3SRC_1 );
      //  __HAL_RCC_PLL3_SOURCE(pll3->PLLSource);

        /* Wait till PLL SOURCE is ready */
        while( (RCC->RCK3SELR & RCC_RCK3SELR_PLL3SRCRDY) != RCC_RCK3SELR_PLL3SRCRDY );

        /* Select PLL3 input reference frequency range */
        MODIFY_REG(RCC->PLL3CFGR1, RCC_PLL3CFGR1_IFRGE, RCC_PLL3CFGR1_IFRGE_1);
    
    /* Configure the PLL3 multiplication and division factors. */
    MODIFY_REG( RCC->PLL3CFGR1, (RCC_PLL3CFGR1_DIVN | RCC_PLL3CFGR1_DIVM3) , ( (plln - 1U) | ( (pllm - 1U) << 16U) ) );
    MODIFY_REG( RCC->PLL3CFGR2, (RCC_PLL3CFGR2_DIVP | RCC_PLL3CFGR2_DIVQ | RCC_PLL3CFGR2_DIVR),( (pllp - 1U) | ( (pllq - 1U) <<8U ) | ( (36U) <<16U) ));
                      
        /* Configure the Fractional Divider */
        CLEAR_BIT(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACLE); //Set FRACLE to ‘0’

        /* Configure PLL  PLL3FRACV  in fractional mode*/
        MODIFY_REG(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACV,(uint32_t)(pllfracv) << RCC_PLL3FRACR_FRACV_Pos);
        
        SET_BIT(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACLE); //Set FRACLE to ‘1’

        CLEAR_BIT(RCC->PLL3CR, RCC_PLL3CR_SSCG_CTRL);

        /* Enable the PLL3. */
        SET_BIT(RCC->PLL3CR, RCC_PLL3CR_PLLON );

        /* Wait till PLL is ready */
        while((RCC->PLL3CR & RCC_PLL3CR_PLL3RDY) !=  RCC_PLL3CR_PLL3RDY );

        /* Enable the post-dividers */
        SET_BIT(RCC->PLL3CR, RCC_PLL3_DIVP | RCC_PLL3_DIVQ | RCC_PLL3_DIVR );
        //---------- Configure PLL4 -----------------------
        CLEAR_BIT(RCC->PLL4CR, RCC_PLL4_DIVP | RCC_PLL4_DIVQ | RCC_PLL4_DIVR );/*Disable the post-dividers*/
      
      CLEAR_BIT(RCC->PLL4CR, RCC_PLL4CR_PLLON);     /* Disable the main PLL. */

      /* Wait till PLL is ready */
      while((RCC->PLL4CR & RCC_PLL4CR_PLL4RDY) ==  RCC_PLL4CR_PLL4RDY );

      /* Configure PLL4 and PLL4 clock source */
       MODIFY_REG( RCC->RCK4SELR, RCC_RCK4SELR_PLL4SRC, RCC_RCK4SELR_PLL4SRC_1 );
    
      /* Wait till PLL SOURCE is ready */
      while( (RCC->RCK4SELR & RCC_RCK4SELR_PLL4SRCRDY) != RCC_RCK4SELR_PLL4SRCRDY );
    
      /* Select PLL4 input reference frequency range */
      MODIFY_REG(RCC->PLL4CFGR1, RCC_PLL4CFGR1_IFRGE, RCC_PLL4CFGR1_IFRGE_0);

      /* Configure the PLL4 multiplication and division factors. */
      MODIFY_REG( RCC->PLL4CFGR1, (RCC_PLL4CFGR1_DIVN | RCC_PLL4CFGR1_DIVM4) , ( 98U | ( 3U << 16U) ) );
      MODIFY_REG( RCC->PLL4CFGR2, (RCC_PLL4CFGR2_DIVP | RCC_PLL4CFGR2_DIVQ | RCC_PLL4CFGR2_DIVR), ( 5U | ( 7U <<8U ) | ( 7U <<16U) ));
                                   

      /* Configure the Fractional Divider */

    CLEAR_BIT(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACLE);      //Set FRACLE to ‘0’

      
      /* Do not use the fractional divider */
      MODIFY_REG(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACV,(uint32_t)(0) << RCC_PLL4FRACR_FRACV_Pos);
      
     // __HAL_RCC_PLL4FRACV_ENABLE(); 
      SET_BIT(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACLE); //Set FRACLE to ‘1’

      CLEAR_BIT(RCC->PLL4CR, RCC_PLL4CR_SSCG_CTRL);

      /* Enable the PLL4. */
      SET_BIT(RCC->PLL4CR, RCC_PLL4CR_PLLON );
     

      /* Wait till PLL is ready */
      while((RCC->PLL4CR & RCC_PLL4CR_PLL4RDY) !=  RCC_PLL4CR_PLL4RDY );

      /* Enable PLL4P Clock output. */
      SET_BIT(RCC->PLL4CR, RCC_PLL4_DIVP | RCC_PLL4_DIVQ | RCC_PLL4_DIVR );
      
//    if (HAL_RCC_OscConfig(&rcc_osc_init_handle) != HAL_OK)
//    {
//        return 1;
//    }
    
    /**
     * 配置RCC时钟，也就是HCLK、ACLK、PCLK1~5以及MPU。
     * PLL4也分为三路出去：PLL4P、PLL4Q和PLL4R。这三路均从PLL4N配置出来
     *                    Fref4_ck = 24M / PLL4M = 24M / 4 = 6MHz
     *                  PLL4_vco = Fref4_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 6MHz * (99 + (0 / 8192))
     *                             = 594MHz
     */
//    rcc_clk_init_handle.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
//                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
//                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
//    rcc_clk_init_handle.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK3 ;
//    
//    rcc_clk_init_handle.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;        /* MPU时钟源为PLL1P = 650MHz             */
//    rcc_clk_init_handle.MPUInit.MPU_Div = RCC_MPU_DIV2;                /* MPUDIV 2分频 = 650 / 2 = 325MHz        */
//    rcc_clk_init_handle.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;    /* AXI时钟为PLL2P = 266.5MHz             */
//    rcc_clk_init_handle.AXISSInit.AXI_Div = RCC_AXI_DIV1;            /* AXI时钟1分频                         */
//    rcc_clk_init_handle.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;    /* MCU时钟源选择PLL3P=209MHz             */
//    rcc_clk_init_handle.MCUInit.MCU_Div = RCC_MCU_DIV1;                /* MCU时钟1分频                         */
//    rcc_clk_init_handle.APB4_Div = RCC_APB4_DIV2;                    /* APB4为AXI的2分频=266.5/2=133.25MHz    */                
//    rcc_clk_init_handle.APB5_Div = RCC_APB5_DIV4;                    /* APB5为AXI的4分频=266.5/4=66.625MHz     */
//    rcc_clk_init_handle.APB1_Div = RCC_APB1_DIV2;                    /* APB1为MLHCLK的2分频=209/2=104.5MHz     */
//    rcc_clk_init_handle.APB2_Div = RCC_APB2_DIV2;                    /* APB2为MLHCLK的2分频=209/2=104.5MHz     */
//    rcc_clk_init_handle.APB3_Div = RCC_APB3_DIV2;                    /* APB3为MLHCLK的2分频=209/2=104.5MHz     */
    
    /* Set MCU clock source */
  MODIFY_REG( RCC->MSSCKSELR, RCC_MSSCKSELR_MCUSSRC , RCC_MSSCKSELR_MCUSSRC_3 );
  /* Wait till MCU is ready */
//  while (__HAL_RCC_GET_FLAG(RCC_FLAG_MCUSSRCRDY) == RESET);
  while( (RCC->MSSCKSELR & RCC_MSSCKSELR_MCUSSRCRDY)  != RCC_MSSCKSELR_MCUSSRCRDY );

  /* Set MCU division factor */
  MODIFY_REG( RCC->MCUDIVR, RCC_MCUDIVR_MCUDIV , RCC_MCUDIVR_MCUDIV_0 );
  /* Wait till MCU is ready */
 // while (__HAL_RCC_GET_FLAG(RCC_FLAG_MCUDIVRDY) == RESET);
  while( (RCC->MCUDIVR & RCC_MCUDIVR_MCUDIVRDY) != RCC_MCUDIVR_MCUDIVRDY );
    
        /* Set APB1 division factor */
       MODIFY_REG( RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV , RCC_APB1DIVR_APB1DIV_1 );
 //   while (__HAL_RCC_GET_FLAG(RCC_FLAG_APB1DIVRDY) == RESET);
    while((RCC->APB1DIVR & RCC_APB1DIVR_APB1DIVRDY) !=  RCC_APB1DIVR_APB1DIVRDY );/* Wait till APB1 is ready */
    
        /* Set APB2 division factor */
      MODIFY_REG( RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV , RCC_APB2DIVR_APB2DIV_1 ); 

    /* Wait till APB2 is ready */
 //   while (__HAL_RCC_GET_FLAG(RCC_FLAG_APB2DIVRDY) == RESET);
    while((RCC->APB2DIVR & RCC_APB2DIVR_APB2DIVRDY) !=  RCC_APB2DIVR_APB2DIVRDY );
    
        /* Set APB3 division factor */
      MODIFY_REG( RCC->APB3DIVR, RCC_APB3DIVR_APB3DIV , RCC_APB3DIVR_APB3DIV_1 );

    /* Wait till APB3 is ready */
//    while (__HAL_RCC_GET_FLAG(RCC_FLAG_APB3DIVRDY) == RESET);
    while((RCC->APB3DIVR & RCC_APB3DIVR_APB3DIVRDY) !=  RCC_APB3DIVR_APB3DIVRDY );
    
//    if (HAL_RCC_ClockConfig(&rcc_clk_init_handle) != HAL_OK)
//    {
//        return 2;
//    }

    return 0;
}

#if defined(__clang__) /* 使用V6编译器(clang) */

/* THUMB指令不支持汇编内联 */
/* 采用如下方法实现执行汇编指令WFI */
void __attribute__((noinline)) WFI_SET(void)
{
    __asm__("wfi");
}

/* 关闭所有中断(但是不包括fault和NMI中断) */
void __attribute__((noinline)) INTX_DISABLE(void)
{
    __asm__("cpsid i \t\n"
            "bx lr");
}

/* 开启所有中断 */
void __attribute__((noinline)) INTX_ENABLE(void)
{
    __asm__("cpsie i \t\n"
            "bx lr");
}

/* 设置栈顶地址 */
/* addr:栈顶地址 */
void __attribute__((noinline)) MSR_MSP(uint32_t addr) 
{
    __asm__("msr msp, r0 \t\n"
            "bx r14");
}

#elif defined (__CC_ARM)    /* 使用V5编译器(ARMCC) */

/* THUMB指令不支持汇编内联 */
/* 采用如下方法实现执行汇编指令WFI */
__asm void WFI_SET(void)
{
    WFI;
}

/* 关闭所有中断(但是不包括fault和NMI中断) */
__asm void INTX_DISABLE(void)
{
    CPSID  I
    BX     LR
}

/* 开启所有中断 */
__asm void INTX_ENABLE(void)
{
    CPSIE  I
    BX     LR
}

/* 设置栈顶地址 */
/* addr:栈顶地址 */
__asm void MSR_MSP(uint32_t addr)
{
    MSR MSP, r0  /* set Main Stack value */
    BX  r14
}

#endif


#ifdef  USE_FULL_ASSERT

/**
 * @brief       当编译提示出错的时候此函数用来报告错误的文件和所在行
 * @param       file：指向源文件
 *              line：指向在文件中的行数
 * @retval      无
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    while (1)
    {
    }
}
#endif
 
