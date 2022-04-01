################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Cntrl_Proc.c \
../Src/battery.c \
../Src/charge.c \
../Src/etc.c \
../Src/led.c \
../Src/lidar.c \
../Src/lift.c \
../Src/main.c \
../Src/nuc.c \
../Src/optcom.c \
../Src/sound.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c \
../Src/task_charger_link.c \
../Src/tbt.c \
../Src/turn.c \
../Src/wheel.c \
../Src/wheel_test.c 

OBJS += \
./Src/Cntrl_Proc.o \
./Src/battery.o \
./Src/charge.o \
./Src/etc.o \
./Src/led.o \
./Src/lidar.o \
./Src/lift.o \
./Src/main.o \
./Src/nuc.o \
./Src/optcom.o \
./Src/sound.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o \
./Src/task_charger_link.o \
./Src/tbt.o \
./Src/turn.o \
./Src/wheel.o \
./Src/wheel_test.o 

C_DEPS += \
./Src/Cntrl_Proc.d \
./Src/battery.d \
./Src/charge.d \
./Src/etc.d \
./Src/led.d \
./Src/lidar.d \
./Src/lift.d \
./Src/main.d \
./Src/nuc.d \
./Src/optcom.d \
./Src/sound.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d \
./Src/task_charger_link.d \
./Src/tbt.d \
./Src/turn.d \
./Src/wheel.d \
./Src/wheel_test.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/katte/Desktop/TitraG1000/Inc" -I"C:/Users/katte/Desktop/TitraG1000/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/katte/Desktop/TitraG1000/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/katte/Desktop/TitraG1000/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/katte/Desktop/TitraG1000/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


