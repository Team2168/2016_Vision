################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/main.cpp \
../src/tcp_client.cpp 

OBJS += \
./src/main.o \
./src/tcp_client.o 

CPP_DEPS += \
./src/main.d \
./src/tcp_client.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabi-g++-4.6 -I"/home/kevin/workspace/2168_Vision/_Includes/OpenCV/2.4.10" -I"/home/kevin/workspace/2168_Vision/_Includes/c++/4.6.3" -O2 -g -Wall -c -fmessage-length=0 -v -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


