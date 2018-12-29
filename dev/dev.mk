# ===========================================================================
# COMMON INCLUDES AND SOURCES
#
#	Add common modules and utilities into this section.
#
# ===========================================================================


DEV_COMMON_CSRC =
DEV_COMMON_CPPSRC = common/port_to_string.cpp \
			        debug/button_monitor.cpp \
			        debug/serial_shell.cpp \
			        debug/serial_shell_commands.cpp \
			        debug/led.cpp
DEV_COMMON_INC = . \
		         common \
		         debug

# ===========================================================================
# MAIN MODULES
# ===========================================================================

DEV_MAIN_CSRC =
DEV_MAIN_CPPSRC = interfaces/remote_interpreter.cpp \
				  interfaces/gimbal_process_function.cpp \
				  interfaces/send_currents_functions.cpp \
			      main.cpp
DEV_MAIN_INC = interfaces


# ===========================================================================
# REMOTE_INTERPRETER MODULES
#
#	Include remote_interpreter sources and the unit test modules.
#
# ===========================================================================

DEV_REMOTE_INTERPRETER_CSRC =
DEV_REMOTE_INTERPRETER_CPPSRC = interfaces/remote_interpreter.cpp \
                                interfaces/remote_interpreter_unit_test.cpp
DEV_REMOTE_INTERPRETER_INC = interfaces

# ===========================================================================
# GIMBAL_INTERFACE MODULES
#
#	Include CANInterface, GimbalInterface and the unit test modules.
#
# ===========================================================================

DEV_GIMBAL_INTERFACE_CSRC =
DEV_GIMBAL_INTERFACE_CPPSRC = interfaces/can_interface.cpp \
                                interfaces/gimbal_interface.cpp \
                                interfaces/gimbal_interface_unit_test.cpp
DEV_GIMBAL_INTERFACE_INC = interfaces


# ===========================================================================
# RULES
#
# 	Add common components and files specifized by parameter DEV_MODULE to
#   the aggregate list of Makefiles.
#
# ===========================================================================
ALLCSRC += $(DEV_COMMON_CSRC) \
           $(DEV_$(DEV_MODULE)_CSRC)
ALLCPPSRC += $(DEV_COMMON_CPPSRC) \
		     $(DEV_$(DEV_MODULE)_CPPSRC)
ALLINC  += $(DEV_COMMON_INC) \
		   $(DEV_$(DEV_MODULE)_INC)