################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_initialize_hardware.c \
../src/botao.c \
../src/buzzer.c \
../src/delays.c \
../src/encoders.c \
../src/flash.c \
../src/flood_fill.c \
../src/leds.c \
../src/main.c \
../src/motores.c \
../src/rodinhas.c \
../src/sensores.c \
../src/speed_profile.c \
../src/stm32f4xx_it.c \
../src/usart1.c \
../src/usb_user.c \
../src/usbd_conf.c \
../src/usbd_desc.c 

OBJS += \
./src/_initialize_hardware.o \
./src/botao.o \
./src/buzzer.o \
./src/delays.o \
./src/encoders.o \
./src/flash.o \
./src/flood_fill.o \
./src/leds.o \
./src/main.o \
./src/motores.o \
./src/rodinhas.o \
./src/sensores.o \
./src/speed_profile.o \
./src/stm32f4xx_it.o \
./src/usart1.o \
./src/usb_user.o \
./src/usbd_conf.o \
./src/usbd_desc.o 

C_DEPS += \
./src/_initialize_hardware.d \
./src/botao.d \
./src/buzzer.d \
./src/delays.d \
./src/encoders.d \
./src/flash.d \
./src/flood_fill.d \
./src/leds.d \
./src/main.d \
./src/motores.d \
./src/rodinhas.d \
./src/sensores.d \
./src/speed_profile.d \
./src/stm32f4xx_it.d \
./src/usart1.d \
./src/usb_user.d \
./src/usbd_conf.d \
./src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra  -g -DSTM32F405xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I../USB/CDC/Inc -I../USB/Core/Inc -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


