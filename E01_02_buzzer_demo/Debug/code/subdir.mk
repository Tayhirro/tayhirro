################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/pico_gy.c \
../code/pico_link_ii.c \
../code/xiao_barrier.c \
../code/xiao_basic_function.c \
../code/xiao_camera_processing.c \
../code/xiao_circle.c \
../code/xiao_cross.c \
../code/xiao_encoder.c \
../code/xiao_grage.c \
../code/xiao_gyroscope.c \
../code/xiao_image_processing.c \
../code/xiao_motor.c \
../code/xiao_pid.c \
../code/xiao_show.c \
../code/xiao_steer.c \
../code/xiao_trace.c \
../code/xiao_vofa.c 

COMPILED_SRCS += \
./code/pico_gy.src \
./code/pico_link_ii.src \
./code/xiao_barrier.src \
./code/xiao_basic_function.src \
./code/xiao_camera_processing.src \
./code/xiao_circle.src \
./code/xiao_cross.src \
./code/xiao_encoder.src \
./code/xiao_grage.src \
./code/xiao_gyroscope.src \
./code/xiao_image_processing.src \
./code/xiao_motor.src \
./code/xiao_pid.src \
./code/xiao_show.src \
./code/xiao_steer.src \
./code/xiao_trace.src \
./code/xiao_vofa.src 

C_DEPS += \
./code/pico_gy.d \
./code/pico_link_ii.d \
./code/xiao_barrier.d \
./code/xiao_basic_function.d \
./code/xiao_camera_processing.d \
./code/xiao_circle.d \
./code/xiao_cross.d \
./code/xiao_encoder.d \
./code/xiao_grage.d \
./code/xiao_gyroscope.d \
./code/xiao_image_processing.d \
./code/xiao_motor.d \
./code/xiao_pid.d \
./code/xiao_show.d \
./code/xiao_steer.d \
./code/xiao_trace.d \
./code/xiao_vofa.d 

OBJS += \
./code/pico_gy.o \
./code/pico_link_ii.o \
./code/xiao_barrier.o \
./code/xiao_basic_function.o \
./code/xiao_camera_processing.o \
./code/xiao_circle.o \
./code/xiao_cross.o \
./code/xiao_encoder.o \
./code/xiao_grage.o \
./code/xiao_gyroscope.o \
./code/xiao_image_processing.o \
./code/xiao_motor.o \
./code/xiao_pid.o \
./code/xiao_show.o \
./code/xiao_steer.o \
./code/xiao_trace.o \
./code/xiao_vofa.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/¿µ/Desktop/smartcar code/tayhirro/E01_02_buzzer_demo/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/pico_gy.d ./code/pico_gy.o ./code/pico_gy.src ./code/pico_link_ii.d ./code/pico_link_ii.o ./code/pico_link_ii.src ./code/xiao_barrier.d ./code/xiao_barrier.o ./code/xiao_barrier.src ./code/xiao_basic_function.d ./code/xiao_basic_function.o ./code/xiao_basic_function.src ./code/xiao_camera_processing.d ./code/xiao_camera_processing.o ./code/xiao_camera_processing.src ./code/xiao_circle.d ./code/xiao_circle.o ./code/xiao_circle.src ./code/xiao_cross.d ./code/xiao_cross.o ./code/xiao_cross.src ./code/xiao_encoder.d ./code/xiao_encoder.o ./code/xiao_encoder.src ./code/xiao_grage.d ./code/xiao_grage.o ./code/xiao_grage.src ./code/xiao_gyroscope.d ./code/xiao_gyroscope.o ./code/xiao_gyroscope.src ./code/xiao_image_processing.d ./code/xiao_image_processing.o ./code/xiao_image_processing.src ./code/xiao_motor.d ./code/xiao_motor.o ./code/xiao_motor.src ./code/xiao_pid.d ./code/xiao_pid.o ./code/xiao_pid.src ./code/xiao_show.d ./code/xiao_show.o ./code/xiao_show.src ./code/xiao_steer.d ./code/xiao_steer.o ./code/xiao_steer.src ./code/xiao_trace.d ./code/xiao_trace.o ./code/xiao_trace.src ./code/xiao_vofa.d ./code/xiao_vofa.o ./code/xiao_vofa.src

.PHONY: clean-code

