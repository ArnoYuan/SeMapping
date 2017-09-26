################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/GMapping/Sensor/SensorOdometry/OdometryReading.cpp \
../Source/GMapping/Sensor/SensorOdometry/OdometrySensor.cpp 

OBJS += \
./Source/GMapping/Sensor/SensorOdometry/OdometryReading.o \
./Source/GMapping/Sensor/SensorOdometry/OdometrySensor.o 

CPP_DEPS += \
./Source/GMapping/Sensor/SensorOdometry/OdometryReading.d \
./Source/GMapping/Sensor/SensorOdometry/OdometrySensor.d 


# Each subdirectory must supply rules for building sources it contributes
Source/GMapping/Sensor/SensorOdometry/%.o: ../Source/GMapping/Sensor/SensorOdometry/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


