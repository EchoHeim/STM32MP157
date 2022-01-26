/**
  @page LPTIM_PulseCounter Low power timer pulse counter example
  
  @verbatim
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    LPTIM/LPTIM_PulseCounter/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the LPTIM Pulse counter example
  ******************************************************************************
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This example describes how to configure and use LPTIM to count pulses through
the LPTIM HAL API.

To reduce power consumption, MCU enters stop mode after starting counting. Each
time the counter reachs the maximum value (Period/Autoreload), an interruption
is generated, the MCU is woke up from stop mode and LED4 toggles the last state.
  
In this example Period value is set to 1000, so each time the counter counts
(1000 + 1) rising edges on LPTIM Input pin PB10, an interrupt is generated and LED4
toggles.

In this example the internal clock provided to the LPTIM2 is LSI (32 Khz),
so the external input is sampled with LSI clock. In order not to miss any event,
the frequency of the changes on the external Input1 signal should never exceed the
frequency of the internal clock provided to the LPTIM2 (LSI for the
present example).

@note This example can not be used in DEBUG mode, this is due to the fact 
      that the Cortex-M4 core is no longer clocked during low power mode 
      so debugging features are disabled.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then 
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note This example needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents  

  - LPTIM/LPTIM_PulseCounter/Inc/stm32mp1xx_hal_conf.h    HAL configuration file
  - LPTIM/LPTIM_PulseCounter/Inc/stm32mp1xx_it.h          Interrupt handlers header file
  - LPTIM/LPTIM_PulseCounter/Inc/main.h                  Header for main.c module  
  - LPTIM/LPTIM_PulseCounter/Src/stm32mp1xx_it.c          Interrupt handlers
  - LPTIM/LPTIM_PulseCounter/Src/main.c                  Main program
  - LPTIM/LPTIM_PulseCounter/Src/stm32mp1xx_hal_msp.c     HAL MSP module
  - LPTIM/LPTIM_PulseCounter/Src/system_stm32mp1xx.c      STM32MP1xx system source file


@par Hardware and Software environment

  - This example runs on STM32MP157CAAx devices.
    
  - This example has been tested with STMicroelectronics STM32MP157C-EV1
    board and can be easily tailored to any other supported device
    and development board.      

  - Generate pulses on PB10 (pin 8 in CN21 connector). (Connect a square waveform).

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example 

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
