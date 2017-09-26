################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/GMapping/ScanMatcher/Eig3.cpp \
../Source/GMapping/ScanMatcher/ScanMatcher.cpp \
../Source/GMapping/ScanMatcher/ScanMatcherMap.cpp \
../Source/GMapping/ScanMatcher/ScanMatcherProcessor.cpp 

OBJS += \
./Source/GMapping/ScanMatcher/Eig3.o \
./Source/GMapping/ScanMatcher/ScanMatcher.o \
./Source/GMapping/ScanMatcher/ScanMatcherMap.o \
./Source/GMapping/ScanMatcher/ScanMatcherProcessor.o 

CPP_DEPS += \
./Source/GMapping/ScanMatcher/Eig3.d \
./Source/GMapping/ScanMatcher/ScanMatcher.d \
./Source/GMapping/ScanMatcher/ScanMatcherMap.d \
./Source/GMapping/ScanMatcher/ScanMatcherProcessor.d 


# Each subdirectory must supply rules for building sources it contributes
Source/GMapping/ScanMatcher/%.o: ../Source/GMapping/ScanMatcher/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


