################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/GMapping/GridFastSlam/GFSReader.cpp \
../Source/GMapping/GridFastSlam/GridSlamProcessor.cpp \
../Source/GMapping/GridFastSlam/GridSlamProcessorTree.cpp \
../Source/GMapping/GridFastSlam/MotionModel.cpp 

OBJS += \
./Source/GMapping/GridFastSlam/GFSReader.o \
./Source/GMapping/GridFastSlam/GridSlamProcessor.o \
./Source/GMapping/GridFastSlam/GridSlamProcessorTree.o \
./Source/GMapping/GridFastSlam/MotionModel.o 

CPP_DEPS += \
./Source/GMapping/GridFastSlam/GFSReader.d \
./Source/GMapping/GridFastSlam/GridSlamProcessor.d \
./Source/GMapping/GridFastSlam/GridSlamProcessorTree.d \
./Source/GMapping/GridFastSlam/MotionModel.d 


# Each subdirectory must supply rules for building sources it contributes
Source/GMapping/GridFastSlam/%.o: ../Source/GMapping/GridFastSlam/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-openwrt-linux-muslgnueabi-g++ -I$(SENAVICOMMON_PATH)/Source -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


