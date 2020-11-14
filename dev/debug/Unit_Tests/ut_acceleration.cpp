//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "can_interface.h"
#include "pid_controller.hpp"
// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface::motor_feedback_t *feedback;
PIDController acc_pid;
PIDController v2acc_pid;
int *targetC;
float targetV;
/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_acc_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "ac_set_pid kp ki kd i_limit out_limit");
        return;
    }
    PIDController::pid_params_t pidParams;
    pidParams.kp = Shell::atof(argv[0]);
    pidParams.ki = Shell::atof(argv[1]);
    pidParams.kd = Shell::atof(argv[2]);
    pidParams.i_limit = Shell::atof(argv[3]);
    pidParams.out_limit = Shell::atof(argv[4]);
    acc_pid.change_parameters(pidParams);
    chprintf(chp, "pid set!" SHELL_NEWLINE_STR);
}

static void cmd_set_vel_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "ve_set_pid kp ki kd i_limit out_limit");
        return;
    }
    PIDController::pid_params_t AccPidParams;
    AccPidParams.kp = Shell::atof(argv[0]);
    AccPidParams.ki = Shell::atof(argv[1]);
    AccPidParams.kd = Shell::atof(argv[2]);
    AccPidParams.i_limit = Shell::atof(argv[3]);
    AccPidParams.out_limit = Shell::atof(argv[4]);
    v2acc_pid.change_parameters(AccPidParams);
    chprintf(chp, "pid set!" SHELL_NEWLINE_STR);
}

static void cmd_fb(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "ac_set_pid kp ki kd i_limit out_limit");
        return;
    }
    Shell::printf("time %u angle %.2f" SHELL_NEWLINE_STR, feedback->last_update_time, feedback->actual_angle);
}

static void cmd_set_v(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "v_set v");
        return;
    }
    targetV = Shell::atof(argv[0]);
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"ac_set_pid", cmd_set_acc_pid},
        {"fb", cmd_fb},
        {"v_set", cmd_set_v},
        {"ve_set_pid", cmd_set_vel_pid},
        {nullptr,    nullptr}
};

int Filter(float NewValue,float old_Value, float FILTER_A) {
    return NewValue * FILTER_A + (1.0 - FILTER_A) * old_Value;
}

// Thread to ...
class pid_Thread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("pid");
        float prev_velocity;
        float filter_Ve;
        float real_acceleration;
        float prev_acceleration;
        float prev_time = SYSTIME;
        int count = 0;
        while (!shouldTerminate()) {
            float target_acc = v2acc_pid.calc(feedback->actual_velocity, targetV);

            filter_Ve = Filter(feedback->actual_velocity, prev_velocity, 0.2);
            real_acceleration = Filter((filter_Ve - prev_velocity)/float(feedback->last_update_time-prev_time)*1000.0, prev_acceleration, 0.8);
            prev_velocity = filter_Ve;
            prev_time = feedback->last_update_time;
            prev_acceleration = real_acceleration;
            *targetC = acc_pid.calc(real_acceleration, 0);
            count += 1;
            if(count >= 5){
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%u,%u" SHELL_NEWLINE_STR,
                              SYSTIME,
                              filter_Ve, 0.0f,
                              real_acceleration, 0.0f,
                              *targetC, 0);
                count = 0;
            }
            sleep(TIME_MS2I(5));
        }
    }
} templateThread;


int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    PIDController::pid_params_t AccPidParams;
    PIDController::pid_params_t VeloPIDParams;
    AccPidParams.kp = 0.1f;
    AccPidParams.ki = 0.1f;
    AccPidParams.kd = 0.5f;
    AccPidParams.i_limit = 1000;
    AccPidParams.out_limit = 3000;
    acc_pid.change_parameters(AccPidParams);

    VeloPIDParams.kp = 5.0f;
    VeloPIDParams.ki = 0.0f;
    VeloPIDParams.kd = 0.0f;
    VeloPIDParams.i_limit = 1000;
    VeloPIDParams.out_limit = 3000;
    v2acc_pid.change_parameters(VeloPIDParams);

    templateThread.start(NORMALPRIO + 1);
    can1.start(NORMALPRIO + 1, NORMALPRIO + 2);
    can1.set_motor_type(0, CANInterface::M3508);
    feedback = can1.get_feedback_address(0);
    targetC = can1.get_target_current_address(0);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
