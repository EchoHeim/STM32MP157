/**
  @page FDCAN_Loopback FDCAN Loopback example

  @verbatim
  ******************************************************************************
  * @file    FDCAN/FDCAN_Loopback/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the FDCAN_Loopback example.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  @endverbatim

@par Example Description

How to configure the FDCAN to operate in loopback mode.

At the beginning of the main program the HAL_Init() function is called to reset
all the peripherals, initialize the systick.
Then the System clock source is configured by the SystemClock_Config() function in case of Engineering Mode, this clock configuration is done
by the Firmware running on the Cortex-A7 in case of Production Mode.

Then:
  - FDCAN module is configured to operate in external loopback mode,
    with Nominal Bit Rate of 1 MBit/s and Data Bit Rate of 8 MBit/s.
  - FDCAN module is configured to receive:
     - messages with pre-defined standard ID to Rx FIFO 0
     - messages with pre-defined extended ID to Rx FIFO 1
  - FDCAN module is started.
  - The following messages are then sent:
     - one standard ID message matching Rx FIFO 0 filter
     - two extended ID messages matching Rx FIFO 1 filter

Main program checks that the three messages are received as expected
If the result is OK, LED7 start blinking
If the messages are not correctly received, or if at any time of the process an error is encountered,
LED7 is turned ON.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Keywords

FDCAN, Loopback, Polling

@par Directory contents

  - FDCAN/FDCAN_Loopback/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - FDCAN/FDCAN_Loopback/Inc/stm32mp1xx_it.h          Header for stm32mp1xx_it.c
  - FDCAN/FDCAN_Loopback/Inc/main.h                  Header for main.c module
  - FDCAN/FDCAN_Loopback/Src/stm32mp1xx_it.c          Interrupt handlers
  - FDCAN/FDCAN_Loopback/Src/main.c                  Main program
  - FDCAN/FDCAN_Loopback/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - FDCAN/FDCAN_Loopback/Src/system_stm32mp1xx.c      stm32mp1xx system source file

@par Hardware and Software environment

  - This example runs on STM32MP157CACx devices.

  - This example has been tested with an STM32MP157C-DK2 board and can be
    easily tailored to any other supported device and development board.
      
@par How to use it ?

In order to make the program work, you must do the following :
  - Open your preferred toolchain
  - Rebuild all files and load your image into target memory
  - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
