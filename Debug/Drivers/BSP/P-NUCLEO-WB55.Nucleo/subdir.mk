################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.c 

OBJS += \
./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.o 

C_DEPS += \
./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/P-NUCLEO-WB55.Nucleo/%.o Drivers/BSP/P-NUCLEO-WB55.Nucleo/%.su Drivers/BSP/P-NUCLEO-WB55.Nucleo/%.cyclo: ../Drivers/BSP/P-NUCLEO-WB55.Nucleo/%.c Drivers/BSP/P-NUCLEO-WB55.Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I"C:/Users/YANGJUNYOUNG/Documents/GitHub/MAIN-VGSI-ZEIT-Elevator-Monitoring-System/Drivers/CAN_SPI_MCP2515" -I"C:/Users/YANGJUNYOUNG/Documents/GitHub/MAIN-VGSI-ZEIT-Elevator-Monitoring-System/Drivers/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-P-2d-NUCLEO-2d-WB55-2e-Nucleo

clean-Drivers-2f-BSP-2f-P-2d-NUCLEO-2d-WB55-2e-Nucleo:
	-$(RM) ./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.cyclo ./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.d ./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.o ./Drivers/BSP/P-NUCLEO-WB55.Nucleo/stm32wbxx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-P-2d-NUCLEO-2d-WB55-2e-Nucleo
