/**
 ****************************************************************************************************
 * @file        sys.c
 * @author      ����ԭ��Linux�Ŷ�(ALIENTEK)
 * @version     V1.1
 * @date        2020-05-04
 * @brief       ϵͳ��ʼ������(����ʱ������/�жϹ���/GPIO���õ�)
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32MP1������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200504
 ****************************************************************************************************
 */
 
#include "sys.h"

/**
 * @brief       M4��Ƶʱ�����ú�����Ҳ��������PLL3
 * @param       plln: PLL3��Ƶϵ��(PLL��Ƶ), ȡֵ��Χ: 25~200.
 * @param       pllm: PLL3Ԥ��Ƶϵ��(��PLL֮ǰ�ķ�Ƶ), ȡֵ��Χ: 1~64.
 * @param       pllp: PLL3��p��Ƶϵ��(PLL֮��ķ�Ƶ), ��Ƶ����Ϊϵͳʱ��, ȡֵ��Χ: 1~128.(�ұ�����2�ı���)
 * @param       pllq: PLL3��q��Ƶϵ��(PLL֮��ķ�Ƶ), ȡֵ��Χ: 1~128.
 * @note
 *
 *              Fvco: VCOƵ��
 *              Fsys: ϵͳʱ��Ƶ��, Ҳ��PLL1��p��Ƶ���ʱ��Ƶ��
 *              Fq:   PLL1��q��Ƶ���ʱ��Ƶ��
 *              Fs:   PLL����ʱ��Ƶ��, ������HSI, CSI, HSE��.
 *              Fvco = (Fs / pllm) * (plln + (pllfracv / 8192))
 *              Fsys = Fvco / pllp;
 *              Fq   = Fvco / pllq;
 *
 *              �ⲿ����Ϊ24M��ʱ��, �Ƽ�ֵ: plln = 34, pllm = 2, pllp = 2, pllq = 17, pllfracv = 6826.
 *              �õ�:Fvco = (24 / 2) * (34 + (6826 / 8192)) = 417.999MHz �� 418MHz
 *                   Fsys = Fvco / pllp = 418 / 2  = 209MHz
 *                   Fq   = Fvco / pllq = 418 / 17 = 24.588MHz
 *
 *              MP157Ĭ����Ҫ���õ�Ƶ������:
 *              CPUƵ��(mcu_ck) = MLHCLK = PLL3P / 1 = 209Mhz
 *              hclk = MLHCLK = 209Mhz
 *              AHB1/2/3/4 = hclk = 209Mhz
 *              APB1/2/3 = MLHCLK / 2 = 104.5Mhz
 * @retval      �������: 0, �ɹ�; 1, ����;
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv)
{
    RCC_OscInitTypeDef rcc_osc_init_handle;
    RCC_ClkInitTypeDef rcc_clk_init_handle;
    
    HAL_PWR_EnableBkUpAccess();                            /* ʹ�ܷ���backup����     */
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);    /* �����޸�LSE             */
    
    /* ��ʼ��CPU��AHB��APB��ʱ�� */
    
    /**
     * ʹ��HSE����HSE(24MHz��Դ����)��ΪPLLʱ��Դ
     */
    rcc_osc_init_handle.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | \
                                        RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_LSI| \
                                        RCC_OSCILLATORTYPE_CSI;
    rcc_osc_init_handle.HSEState = RCC_HSE_ON;        /* ��HSE */
    rcc_osc_init_handle.LSEState = RCC_LSE_ON;        /* ��LSE */
    rcc_osc_init_handle.HSIState = RCC_HSI_ON;        /* ��HSI */
    rcc_osc_init_handle.CSIState = RCC_CSI_ON;        /* ��CSI */
    rcc_osc_init_handle.LSIState = RCC_LSI_ON;        /* ��HSI */
    rcc_osc_init_handle.HSICalibrationValue = 16;
    rcc_osc_init_handle.HSIDivValue = RCC_HSI_DIV1;
    
    /**
     * ����PLL1��PLL1��ҪΪMPUϵͳʱ�ӣ�Ҳ����A7��ʱ��
     * A7ϵͳʱ�Ӽ��㹫ʽΪ��Fref1_ck = 24M / PLLM = 24M / 3 = 8MHz
     *                       PLL1_vco = Fref1_ck * 2 *((DIVN + 1) + (FRACV / 8192))
     *                         PLL1_vco = 8MHz * 2 * (100 + (0/ 8192))     
     *                                = 1600MHz
     */
    rcc_osc_init_handle.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init_handle.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
    rcc_osc_init_handle.PLL.PLLM = 3;
    rcc_osc_init_handle.PLL.PLLN = 100;        /* ע������д���Ǽ�1�Ժ��PLLN(DIVN)ֵ     */
    rcc_osc_init_handle.PLL.PLLP = 1;        /* PLL1P·ʱ��=PLL1_vco / 2 / PLLP = 800MHz     */
    rcc_osc_init_handle.PLL.PLLQ = 1;        /* PLL1Q·ʱ��=PLL1_vco / 2 / PLLQ = 800MHz     */
    rcc_osc_init_handle.PLL.PLLR = 1;        /* PLL1R·ʱ��=PLL1_vco / 2 / PLLR = 800MHz     */
    rcc_osc_init_handle.PLL.PLLFRACV = 0;
    rcc_osc_init_handle.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
    rcc_osc_init_handle.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
    rcc_osc_init_handle.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * ����PLL2��PLL2��ҪΪAXI��AHB5��AHB6��APB4��APB5��GPU��ʱ��
     * PLL2Ҳ��Ϊ��·��ȥ��PLL2P��PLL2Q��PLL2R������·����PLL2N���ó���
     *                    Fref2_ck = 24M / PLL1M = 24M / 3 = 8MHz
     *                  PLL2_vco = Fref2_ck * 2 *((DIVN + 1) + (FRACV / 8192))
     *                             = 8MHz * 2 * (66 + (5120 / 8192))
     *                             = 1066MHz
     */
    rcc_osc_init_handle.PLL2.PLLState = RCC_PLL_ON;                /* ��PLL2 */
    rcc_osc_init_handle.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;    /* PLL2ʱ��ԴΪHSE=24M */
    rcc_osc_init_handle.PLL2.PLLM = 3;
    rcc_osc_init_handle.PLL2.PLLN = 66;        /* ע������д���Ǽ�1�Ժ��PLLN(DIVN)ֵ     */
    rcc_osc_init_handle.PLL2.PLLP = 2;        /* PLL2P·ʱ��=PLL2_vco / 2 / PLLP = 266.5MHz     */
    rcc_osc_init_handle.PLL2.PLLQ = 1;        /* PLL2Q·ʱ��=PLL2_vco / 2 / PLLQ = 533MHz     */
    rcc_osc_init_handle.PLL2.PLLR = 1;        /* PLL2R·ʱ��=PLL2_vco / 2 / PLLR = 533MHz     */
    rcc_osc_init_handle.PLL2.PLLFRACV = 5120;
    rcc_osc_init_handle.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
    rcc_osc_init_handle.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
    rcc_osc_init_handle.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * ����PLL3��PLL3��ҪΪMCU��MCU Systick��AHB1~AHB4��APB1~APB3�����ʱ�ӡ�DFSDMʱ��
     * PLL3Ҳ��Ϊ��·��ȥ��PLL3P��PLL3Q��PLL3R������·����PLL3N���ó���
     *                    Fref3_ck = 24M / PLL3M = 24M / 2 = 12MHz
     *                  PLL3_vco = Fref3_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 12MHz * (34 + (6826 / 8192))
     *                             = 417.999MHz
     *                             �� 418MHz
     */
    rcc_osc_init_handle.PLL3.PLLState = RCC_PLL_ON;                /* ��PLL3 */
    rcc_osc_init_handle.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;    /* PLL3ʱ��ԴΪHSE=24MHz */
    rcc_osc_init_handle.PLL3.PLLM = pllm;    
    rcc_osc_init_handle.PLL3.PLLN = plln;        /* ע������д���Ǽ�1�Ժ��PLLN(DIVN)ֵ     */
    rcc_osc_init_handle.PLL3.PLLP = pllp;        /* PLL3P·ʱ��=PLL3_vco / PLLP = 208.9995MHz     */
    rcc_osc_init_handle.PLL3.PLLQ = pllq;        /* PLL3Q·ʱ��=PLL3_vco / PLLQ = 24.5881MHz     */
    rcc_osc_init_handle.PLL3.PLLR = 37;            /* PLL3R·ʱ��=PLL3_vco / PLLR = 11.2972MHz     */
    rcc_osc_init_handle.PLL3.PLLRGE = RCC_PLL3IFRANGE_1; /* ref3_ck��ΧΪ8~16Mhz */
    rcc_osc_init_handle.PLL3.PLLFRACV = pllfracv;
    rcc_osc_init_handle.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
    rcc_osc_init_handle.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
    rcc_osc_init_handle.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    
    /**
     * ����PLL4��ĳЩ����ʱ��Դ������LTDC��DSI��
     * PLL4Ҳ��Ϊ��·��ȥ��PLL4P��PLL4Q��PLL4R������·����PLL4N���ó���
     *                    Fref4_ck = 24M / PLL4M = 24M / 4 = 6MHz
     *                  PLL4_vco = Fref4_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 6MHz * (99 + (0 / 8192))
     *                             = 594MHz
     */
    rcc_osc_init_handle.PLL4.PLLState = RCC_PLL_ON;                /* ��PLL4 */
    rcc_osc_init_handle.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;    /* PLL4ʱ��ԴΪHSE=24MHz */
    rcc_osc_init_handle.PLL4.PLLM = 4;
    rcc_osc_init_handle.PLL4.PLLN = 99;            /* ע������д���Ǽ�1�Ժ��PLLN(DIVN)ֵ     */
    rcc_osc_init_handle.PLL4.PLLP = 6;            /* PLL4P·ʱ��=PLL4_vco / PLLP = 99MHz     */
    rcc_osc_init_handle.PLL4.PLLQ = 8;            /* PLL4Q·ʱ��=PLL4_vco / PLLQ = 74.25MHz     */
    rcc_osc_init_handle.PLL4.PLLR = 8;            /* PLL4R·ʱ��=PLL4_vco / PLLR = 74.25MHz     */
    rcc_osc_init_handle.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
    rcc_osc_init_handle.PLL4.PLLFRACV = 0;
    rcc_osc_init_handle.PLL4.PLLMODE = RCC_PLL_INTEGER;
    rcc_osc_init_handle.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
    rcc_osc_init_handle.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
    if (HAL_RCC_OscConfig(&rcc_osc_init_handle) != HAL_OK)
    {
        return 1;
    }
    
    /**
     * ����RCCʱ�ӣ�Ҳ����HCLK��ACLK��PCLK1~5�Լ�MPU��
     * PLL4Ҳ��Ϊ��·��ȥ��PLL4P��PLL4Q��PLL4R������·����PLL4N���ó���
     *                    Fref4_ck = 24M / PLL4M = 24M / 4 = 6MHz
     *                  PLL4_vco = Fref4_ck * ((DIVN + 1) + (FRACV / 8192))
     *                             = 6MHz * (99 + (0 / 8192))
     *                             = 594MHz
     */
    rcc_clk_init_handle.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
    rcc_clk_init_handle.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;        /* MPUʱ��ԴΪPLL1P = 650MHz             */
    rcc_clk_init_handle.MPUInit.MPU_Div = RCC_MPU_DIV2;                /* MPUDIV 2��Ƶ = 650 / 2 = 325MHz        */
    rcc_clk_init_handle.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;    /* AXIʱ��ΪPLL2P = 266.5MHz             */
    rcc_clk_init_handle.AXISSInit.AXI_Div = RCC_AXI_DIV1;            /* AXIʱ��1��Ƶ                         */
    rcc_clk_init_handle.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;    /* MCUʱ��Դѡ��PLL3P=209MHz             */
    rcc_clk_init_handle.MCUInit.MCU_Div = RCC_MCU_DIV1;                /* MCUʱ��1��Ƶ                         */
    rcc_clk_init_handle.APB4_Div = RCC_APB4_DIV2;                    /* APB4ΪAXI��2��Ƶ=266.5/2=133.25MHz    */                
    rcc_clk_init_handle.APB5_Div = RCC_APB5_DIV4;                    /* APB5ΪAXI��4��Ƶ=266.5/4=66.625MHz     */
    rcc_clk_init_handle.APB1_Div = RCC_APB1_DIV2;                    /* APB1ΪMLHCLK��2��Ƶ=209/2=104.5MHz     */
    rcc_clk_init_handle.APB2_Div = RCC_APB2_DIV2;                    /* APB2ΪMLHCLK��2��Ƶ=209/2=104.5MHz     */
    rcc_clk_init_handle.APB3_Div = RCC_APB3_DIV2;                    /* APB3ΪMLHCLK��2��Ƶ=209/2=104.5MHz     */
    if (HAL_RCC_ClockConfig(&rcc_clk_init_handle) != HAL_OK)
    {
        return 2;
    }

    return 0;
}

#if defined(__clang__) /* ʹ��V6������(clang) */

/* THUMBָ�֧�ֻ������ */
/* �������·���ʵ��ִ�л��ָ��WFI */
void __attribute__((noinline)) WFI_SET(void)
{
    __asm__("wfi");
}

/* �ر������ж�(���ǲ�����fault��NMI�ж�) */
void __attribute__((noinline)) INTX_DISABLE(void)
{
    __asm__("cpsid i \t\n"
            "bx lr");
}

/* ���������ж� */
void __attribute__((noinline)) INTX_ENABLE(void)
{
    __asm__("cpsie i \t\n"
            "bx lr");
}

/* ����ջ����ַ */
/* addr:ջ����ַ */
void __attribute__((noinline)) MSR_MSP(uint32_t addr) 
{
    __asm__("msr msp, r0 \t\n"
            "bx r14");
}

#elif defined (__CC_ARM)    /* ʹ��V5������(ARMCC) */

/* THUMBָ�֧�ֻ������ */
/* �������·���ʵ��ִ�л��ָ��WFI */
__asm void WFI_SET(void)
{
    WFI;
}

/* �ر������ж�(���ǲ�����fault��NMI�ж�) */
__asm void INTX_DISABLE(void)
{
    CPSID  I
    BX     LR
}

/* ���������ж� */
__asm void INTX_ENABLE(void)
{
    CPSIE  I
    BX     LR
}

/* ����ջ����ַ */
/* addr:ջ����ַ */
__asm void MSR_MSP(uint32_t addr)
{
    MSR MSP, r0  /* set Main Stack value */
    BX  r14
}

#endif


#ifdef  USE_FULL_ASSERT

/**
 * @brief       ��������ʾ�����ʱ��˺����������������ļ���������
 * @param       file��ָ��Դ�ļ�
 *              line��ָ�����ļ��е�����
 * @retval      ��
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    while (1)
    {
    }
}
#endif 
