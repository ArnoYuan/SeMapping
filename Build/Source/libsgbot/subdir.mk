################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/libsgbot/SgbotApplication.cpp 

OBJS += \
./Source/libsgbot/SgbotApplication.o 

CPP_DEPS += \
./Source/libsgbot/SgbotApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/libsgbot/%.o: ../Source/libsgbot/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -I$(STAGING_DIR/usr/include/libsgbot) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


