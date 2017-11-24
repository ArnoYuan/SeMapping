################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/GMapping/Utils/Movement.cpp \
../Source/GMapping/Utils/PrintMemUsage.cpp \
../Source/GMapping/Utils/Stat.cpp 

OBJS += \
./Source/GMapping/Utils/Movement.o \
./Source/GMapping/Utils/PrintMemUsage.o \
./Source/GMapping/Utils/Stat.o 

CPP_DEPS += \
./Source/GMapping/Utils/Movement.d \
./Source/GMapping/Utils/PrintMemUsage.d \
./Source/GMapping/Utils/Stat.d 


# Each subdirectory must supply rules for building sources it contributes
Source/GMapping/Utils/%.o: ../Source/GMapping/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


