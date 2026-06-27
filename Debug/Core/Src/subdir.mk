################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/can_proto.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/mcp2515.c \
../Core/Src/state.c \
../Core/Src/stepper.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tmc5160.c \
../Core/Src/trajectory.c 

OBJS += \
./Core/Src/can_proto.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/mcp2515.o \
./Core/Src/state.o \
./Core/Src/stepper.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tmc5160.o \
./Core/Src/trajectory.o 

C_DEPS += \
./Core/Src/can_proto.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/mcp2515.d \
./Core/Src/state.d \
./Core/Src/stepper.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tmc5160.d \
./Core/Src/trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/can_proto.cyclo ./Core/Src/can_proto.d ./Core/Src/can_proto.o ./Core/Src/can_proto.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mcp2515.cyclo ./Core/Src/mcp2515.d ./Core/Src/mcp2515.o ./Core/Src/mcp2515.su ./Core/Src/state.cyclo ./Core/Src/state.d ./Core/Src/state.o ./Core/Src/state.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tmc5160.cyclo ./Core/Src/tmc5160.d ./Core/Src/tmc5160.o ./Core/Src/tmc5160.su ./Core/Src/trajectory.cyclo ./Core/Src/trajectory.d ./Core/Src/trajectory.o ./Core/Src/trajectory.su

.PHONY: clean-Core-2f-Src

