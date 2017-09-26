################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Hector/HectorMappingApplication.cpp 

OBJS += \
./Source/Hector/HectorMappingApplication.o 

CPP_DEPS += \
./Source/Hector/HectorMappingApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Hector/%.o: ../Source/Hector/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


