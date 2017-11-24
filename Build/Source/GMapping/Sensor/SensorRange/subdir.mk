################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/GMapping/Sensor/SensorRange/RangeReading.cpp \
../Source/GMapping/Sensor/SensorRange/RangeSensor.cpp 

OBJS += \
./Source/GMapping/Sensor/SensorRange/RangeReading.o \
./Source/GMapping/Sensor/SensorRange/RangeSensor.o 

CPP_DEPS += \
./Source/GMapping/Sensor/SensorRange/RangeReading.d \
./Source/GMapping/Sensor/SensorRange/RangeSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Source/GMapping/Sensor/SensorRange/%.o: ../Source/GMapping/Sensor/SensorRange/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


