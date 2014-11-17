CC := arm-linux-gnueabi-g++

RM := rm -rf


# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../main.cpp \
../tcp_client.cpp \
../testCurl.cpp 

OBJS += \
./main.o \
./tcp_client.o \
./testCurl.o 

CPP_DEPS += \
./main.d \
./tcp_client.d \
./testCurl.d 

USER_OBJS :=

LIBS := -lopencv_highgui -lopencv_core -lopencv_imgproc -lcurl -lpthread -lrt




# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	$(CC) -I/home/OpenCVArm/opencv/platforms/linux/build_softfp/install/include/ -I/usr/arm-linux-gnueabi/include/c++/4.6.3 -I/home/CurlArm/curl/include/curl -O0 -g3 -Wall -c -fmessage-length=0   -mfpu=neon -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
	
	
# All Target
all: BeagleVision

# Tool invocations
BeagleVision: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	$(CC) -L/home/OpenCVArm/opencv/platforms/linux/build_softfp/install/lib -L/usr/arm-linux-gnueabi/lib -L/usr/local/lib -o "BeagleVision" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) BeagleVision
	-@echo ' '	