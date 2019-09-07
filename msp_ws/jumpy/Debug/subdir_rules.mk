################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.cpp $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/lucas/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/lucas/ti/ccsv8/ccs_base/arm/include" --include_path="/home/lucas/catkin_ws/src/jumpy/msp_ws/jumpy/driverlib/MSP432P4xx" --include_path="/home/lucas/catkin_ws/src/jumpy/mcu_uart/include/mcu_uart" --include_path="/home/lucas/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/home/lucas/catkin_ws/src/jumpy/msp_ws/jumpy" --include_path="/home/lucas/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/home/lucas/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/lucas/ti/ccsv8/ccs_base/arm/include" --include_path="/home/lucas/catkin_ws/src/jumpy/msp_ws/jumpy/driverlib/MSP432P4xx" --include_path="/home/lucas/catkin_ws/src/jumpy/mcu_uart/include/mcu_uart" --include_path="/home/lucas/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/home/lucas/catkin_ws/src/jumpy/msp_ws/jumpy" --include_path="/home/lucas/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


