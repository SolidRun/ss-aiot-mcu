################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/acc_sensor.c \
../Core/Src/bq2568.c \
../Core/Src/i2c_slave.c \
../Core/Src/ir_sensor.c \
../Core/Src/ism330dhcx.c \
../Core/Src/ism330dhcx_reg.c \
../Core/Src/main.c \
../Core/Src/protocol.c \
../Core/Src/rtc.c \
../Core/Src/sths34pf80_reg.c \
../Core/Src/stm32u0xx_hal_msp.c \
../Core/Src/stm32u0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u0xx.c \
../Core/Src/ublox.c 

OBJS += \
./Core/Src/acc_sensor.o \
./Core/Src/bq2568.o \
./Core/Src/i2c_slave.o \
./Core/Src/ir_sensor.o \
./Core/Src/ism330dhcx.o \
./Core/Src/ism330dhcx_reg.o \
./Core/Src/main.o \
./Core/Src/protocol.o \
./Core/Src/rtc.o \
./Core/Src/sths34pf80_reg.o \
./Core/Src/stm32u0xx_hal_msp.o \
./Core/Src/stm32u0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u0xx.o \
./Core/Src/ublox.o 

C_DEPS += \
./Core/Src/acc_sensor.d \
./Core/Src/bq2568.d \
./Core/Src/i2c_slave.d \
./Core/Src/ir_sensor.d \
./Core/Src/ism330dhcx.d \
./Core/Src/ism330dhcx_reg.d \
./Core/Src/main.d \
./Core/Src/protocol.d \
./Core/Src/rtc.d \
./Core/Src/sths34pf80_reg.d \
./Core/Src/stm32u0xx_hal_msp.d \
./Core/Src/stm32u0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u0xx.d \
./Core/Src/ublox.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U031xx -c -I../Core/Inc -IC:/Users/User/STM32Cube/Repository/STM32Cube_FW_U0_V1.3.0/Drivers/STM32U0xx_HAL_Driver/Inc -IC:/Users/User/STM32Cube/Repository/STM32Cube_FW_U0_V1.3.0/Drivers/STM32U0xx_HAL_Driver/Inc/Legacy -IC:/Users/User/STM32Cube/Repository/STM32Cube_FW_U0_V1.3.0/Drivers/CMSIS/Device/ST/STM32U0xx/Include -IC:/Users/User/STM32Cube/Repository/STM32Cube_FW_U0_V1.3.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/acc_sensor.cyclo ./Core/Src/acc_sensor.d ./Core/Src/acc_sensor.o ./Core/Src/acc_sensor.su ./Core/Src/bq2568.cyclo ./Core/Src/bq2568.d ./Core/Src/bq2568.o ./Core/Src/bq2568.su ./Core/Src/i2c_slave.cyclo ./Core/Src/i2c_slave.d ./Core/Src/i2c_slave.o ./Core/Src/i2c_slave.su ./Core/Src/ir_sensor.cyclo ./Core/Src/ir_sensor.d ./Core/Src/ir_sensor.o ./Core/Src/ir_sensor.su ./Core/Src/ism330dhcx.cyclo ./Core/Src/ism330dhcx.d ./Core/Src/ism330dhcx.o ./Core/Src/ism330dhcx.su ./Core/Src/ism330dhcx_reg.cyclo ./Core/Src/ism330dhcx_reg.d ./Core/Src/ism330dhcx_reg.o ./Core/Src/ism330dhcx_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/protocol.cyclo ./Core/Src/protocol.d ./Core/Src/protocol.o ./Core/Src/protocol.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/sths34pf80_reg.cyclo ./Core/Src/sths34pf80_reg.d ./Core/Src/sths34pf80_reg.o ./Core/Src/sths34pf80_reg.su ./Core/Src/stm32u0xx_hal_msp.cyclo ./Core/Src/stm32u0xx_hal_msp.d ./Core/Src/stm32u0xx_hal_msp.o ./Core/Src/stm32u0xx_hal_msp.su ./Core/Src/stm32u0xx_it.cyclo ./Core/Src/stm32u0xx_it.d ./Core/Src/stm32u0xx_it.o ./Core/Src/stm32u0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u0xx.cyclo ./Core/Src/system_stm32u0xx.d ./Core/Src/system_stm32u0xx.o ./Core/Src/system_stm32u0xx.su ./Core/Src/ublox.cyclo ./Core/Src/ublox.d ./Core/Src/ublox.o ./Core/Src/ublox.su

.PHONY: clean-Core-2f-Src

