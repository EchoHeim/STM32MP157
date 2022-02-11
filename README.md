# =========== stm32mp157 openSTlinux5.4 ===========

	/***********************************************************\
	|    STM32MP157 内核操作记录,在正点原子出厂系统上修改          |
	|                                                           |
	|    linux-5.4\arch\arm\boot\dts\stm32mp157d-biqu.dts       |
	|    linux-5.4\arch\arm\boot\dts\stm32mp157d-biqu.dtsi      |
	|                                                           |
	\***********************************************************/

## ---- 2022-02-09 ----

01、添加触摸驱动 ft5426;

02、去掉终端光标闪烁;

    修改文件:\drivers\video\fbdev\core\fbcon.c
    去掉光标显示：
    将函数static void fbcon_cursor(struct vc_data *vc, int mode) 改为空函数即可；
    去掉光标闪烁：
    将函数static void fb_flashcursor(struct work_struct *work) 改为空函数。

## ---- 2022-01-25 ----

01、添加 CAN 驱动;

## ---- 2022-01-18 ----

01、添加 Biqu 主板硬件;

## ---- 2022-01-05 ----

01、禁用 can、HDMI;

02、串口只保留 uart4(调试)、uart5;

03、禁用 adc、dac;

04、内核配置，添加 rtl8723ds wifi驱动;

## ---- 2021-11-16 ----

01、禁用 spi1 ，icm20608 陀螺仪;

## ---- 2021-11-13 ----

01、串口修改,屏蔽UART7（蓝牙）和USART3(RS232);

	aliases {
		ethernet0 = &ethernet0;
		serial0 = &uart4;       // console;
		serial1 = &uart5;       // 和 M4 内核通信;
    //	serial2 = &usart3;
	//	serial3 = &uart7;
	};
02、设备树禁用 OV5640、禁用音频设备驱动;

03、禁用 i2C4;
