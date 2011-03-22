################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../example/cameraTest.cpp 

OBJS += \
./example/cameraTest.o 

CPP_DEPS += \
./example/cameraTest.d 


# Each subdirectory must supply rules for building sources it contributes
example/%.o: ../example/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/home/yasir/Documents/code/libraries/eigen -I/opt/ros/unstable/stacks/vision_opencv/opencv2/opencv/include -I/home/yasir/CODE/KinectUnizar/ZGZ/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


