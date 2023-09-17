################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CAN_SPI_MCP2515/CAN_TxRx.c \
../Drivers/CAN_SPI_MCP2515/mcp2515.c 

OBJS += \
./Drivers/CAN_SPI_MCP2515/CAN_TxRx.o \
./Drivers/CAN_SPI_MCP2515/mcp2515.o 

C_DEPS += \
./Drivers/CAN_SPI_MCP2515/CAN_TxRx.d \
./Drivers/CAN_SPI_MCP2515/mcp2515.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CAN_SPI_MCP2515/%.o Drivers/CAN_SPI_MCP2515/%.su Drivers/CAN_SPI_MCP2515/%.cyclo: ../Drivers/CAN_SPI_MCP2515/%.c Drivers/CAN_SPI_MCP2515/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I"C:/Users/YANGJUNYOUNG/Documents/GitHub/MAIN-VGSI-ZEIT-Elevator-Monitoring-System/Drivers/CAN_SPI_MCP2515" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CAN_SPI_MCP2515

clean-Drivers-2f-CAN_SPI_MCP2515:
	-$(RM) ./Drivers/CAN_SPI_MCP2515/CAN_TxRx.cyclo ./Drivers/CAN_SPI_MCP2515/CAN_TxRx.d ./Drivers/CAN_SPI_MCP2515/CAN_TxRx.o ./Drivers/CAN_SPI_MCP2515/CAN_TxRx.su ./Drivers/CAN_SPI_MCP2515/mcp2515.cyclo ./Drivers/CAN_SPI_MCP2515/mcp2515.d ./Drivers/CAN_SPI_MCP2515/mcp2515.o ./Drivers/CAN_SPI_MCP2515/mcp2515.su

.PHONY: clean-Drivers-2f-CAN_SPI_MCP2515

