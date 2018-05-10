################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Source/libsgbot/subdir.mk
-include Source/Hector/Utils/subdir.mk
-include Source/Hector/subdir.mk
-include Source/GMapping/Utils/subdir.mk
-include Source/GMapping/Sensor/SensorRange/subdir.mk
-include Source/GMapping/Sensor/SensorOdometry/subdir.mk
-include Source/GMapping/Sensor/SensorBase/subdir.mk
-include Source/GMapping/ScanMatcher/subdir.mk
-include Source/GMapping/Log/subdir.mk
-include Source/GMapping/GridFastSlam/subdir.mk
-include Source/GMapping/subdir.mk
-include Source/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SeMapping

# Tool invocations
SeMapping: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	#g++ -L$(SENAVICOMMON_PATH)/Build -L$(TOPDIR)/prebuilt/gcc/linux-x86/arm/toolchain-sunxi/toolchain/lib -L$(STAGING_DIR)/usr/lib -o "SeMapping" $(OBJS) $(USER_OBJS) $(LIBS)
	arm-openwrt-linux-muslgnueabi-g++ -L$(SENAVICOMMON_PATH)/Build -L$(TOPDIR)/prebuilt/gcc/linux-x86/arm/toolchain-sunxi/toolchain/lib -L$(STAGING_DIR)/usr/lib -o "SeMapping" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) SeMapping
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
