################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Uart\ Driver/Src/gpio_driver.c \
../Uart\ Driver/Src/ring_buffer.c \
../Uart\ Driver/Src/uart_driver.c 

OBJS += \
./Uart\ Driver/Src/gpio_driver.o \
./Uart\ Driver/Src/ring_buffer.o \
./Uart\ Driver/Src/uart_driver.o 

C_DEPS += \
./Uart\ Driver/Src/gpio_driver.d \
./Uart\ Driver/Src/ring_buffer.d \
./Uart\ Driver/Src/uart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Uart\ Driver/Src/gpio_driver.o: ../Uart\ Driver/Src/gpio_driver.c Uart\ Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/emir_/STM32CubeIDE/workspace_1.16.0/uart_driver_example/Uart Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Uart Driver/Src/gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Uart\ Driver/Src/ring_buffer.o: ../Uart\ Driver/Src/ring_buffer.c Uart\ Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/emir_/STM32CubeIDE/workspace_1.16.0/uart_driver_example/Uart Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Uart Driver/Src/ring_buffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Uart\ Driver/Src/uart_driver.o: ../Uart\ Driver/Src/uart_driver.c Uart\ Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/emir_/STM32CubeIDE/workspace_1.16.0/uart_driver_example/Uart Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Uart Driver/Src/uart_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Uart-20-Driver-2f-Src

clean-Uart-20-Driver-2f-Src:
	-$(RM) ./Uart\ Driver/Src/gpio_driver.cyclo ./Uart\ Driver/Src/gpio_driver.d ./Uart\ Driver/Src/gpio_driver.o ./Uart\ Driver/Src/gpio_driver.su ./Uart\ Driver/Src/ring_buffer.cyclo ./Uart\ Driver/Src/ring_buffer.d ./Uart\ Driver/Src/ring_buffer.o ./Uart\ Driver/Src/ring_buffer.su ./Uart\ Driver/Src/uart_driver.cyclo ./Uart\ Driver/Src/uart_driver.d ./Uart\ Driver/Src/uart_driver.o ./Uart\ Driver/Src/uart_driver.su

.PHONY: clean-Uart-20-Driver-2f-Src

