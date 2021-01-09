################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_HOST/App/usb_host.c 

OBJS += \
./USB_HOST/App/usb_host.o 

C_DEPS += \
./USB_HOST/App/usb_host.d 


# Each subdirectory must supply rules for building sources it contributes
USB_HOST/App/usb_host.o: ../USB_HOST/App/usb_host.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Middlewares/ST/STM32_USB_Host_Library/Core/Inc -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/shnee/STM32Cube/Repository/STM32Cube_FW_L4_V1.16.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"USB_HOST/App/usb_host.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

