################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Camera/camera.cpp 

OBJS += \
./src/Camera/camera.o 

CPP_DEPS += \
./src/Camera/camera.d 


# Each subdirectory must supply rules for building sources it contributes
src/Camera/%.o: ../src/Camera/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/yasir/Documents/code/libraries/eigen -I/home/yasir/CODE/KinectUnizar/ZGZ/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


