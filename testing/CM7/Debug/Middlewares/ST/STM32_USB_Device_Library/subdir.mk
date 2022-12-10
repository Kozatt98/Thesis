################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_core.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_core.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.d \
./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o: C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_core.o: C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o: C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o: C:/Users/kozat/STM32CubeIDE/workspace_1.10.1/testing/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c Middlewares/ST/STM32_USB_Device_Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_cdc.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_core.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_ctlreq.su ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.d ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.o ./Middlewares/ST/STM32_USB_Device_Library/usbd_ioreq.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library

