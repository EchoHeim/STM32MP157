################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma_ex.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_exti.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_gpio.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hsem.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_mdma.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr_ex.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc_ex.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim.c \
D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_cortex.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma_ex.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_exti.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_gpio.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_hsem.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_mdma.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr_ex.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc_ex.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim.o \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_cortex.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma_ex.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_exti.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_gpio.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_hsem.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_mdma.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr_ex.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc_ex.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim.d \
./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_cortex.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma_ex.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma_ex.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_exti.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_exti.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_gpio.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_gpio.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_hsem.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hsem.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_mdma.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_mdma.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr_ex.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr_ex.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc_ex.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc_ex.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim_ex.o: D:/STM32MP157/MP157_M4/CubeIDE/Template/Template/Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim_ex.c Drivers/STM32MP1xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32MP157Dxx -c -I../Core/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc -I../../Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32MP1xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32MP1xx_HAL_Driver

clean-Drivers-2f-STM32MP1xx_HAL_Driver:
	-$(RM) ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_cortex.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_cortex.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma_ex.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_dma_ex.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_exti.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_exti.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_gpio.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_gpio.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_hsem.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_hsem.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_mdma.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_mdma.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr_ex.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_pwr_ex.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc_ex.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_rcc_ex.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim.o ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim_ex.d ./Drivers/STM32MP1xx_HAL_Driver/stm32mp1xx_hal_tim_ex.o

.PHONY: clean-Drivers-2f-STM32MP1xx_HAL_Driver

