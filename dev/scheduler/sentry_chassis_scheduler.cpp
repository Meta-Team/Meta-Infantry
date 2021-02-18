//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
// Modified by Laixinyi on 2019/7/1
//

#include "sentry_chassis_scheduler.h"

SChassisSKD::mode_t SChassisSKD::mode = FORCED_RELAX_MODE;
SChassisSKD::install_direction_t SChassisSKD::motor_install_[2];

PIDController SChassisSKD::a2v_pid;
PIDController SChassisSKD::v2i_pid[MOTOR_COUNT];

float SChassisSKD::target_position = 0;  // [cm], bigger position will drive the chassis towards right
float SChassisSKD::target_velocity = 0;  // [cm/s], calculated target velocity, middle value
int SChassisSKD::target_current[MOTOR_COUNT] = {0, 0};

SChassisSKD::SKDThread SChassisSKD::skdThread;


void SChassisSKD::start(SChassisSKD::install_direction_t left_motor_install,
                        SChassisSKD::install_direction_t right_motor_install, tprio_t thread_prio) {

    motor_install_[MOTOR_LEFT] = left_motor_install;
    motor_install_[MOTOR_RIGHT] = right_motor_install;

    skdThread.start(thread_prio);
}

void SChassisSKD::load_pid_params(PIDControllerBase::pid_params_t sentry_a2v_params,
                                  PIDControllerBase::pid_params_t sentry_v2i_params) {
    v2i_pid[MOTOR_LEFT].change_parameters(sentry_v2i_params);
    v2i_pid[MOTOR_RIGHT].change_parameters(sentry_v2i_params);
    a2v_pid.change_parameters(sentry_a2v_params);
}

void SChassisSKD::reset_origin() {
    SChassisIF::clear_position();
    target_position = 0;
}

void SChassisSKD::set_mode(SChassisSKD::mode_t target_mode) {
    mode = target_mode;
}

//SChassisSKD::mode_t SChassisSKD::get_mode() {
//    return mode;
//}

void SChassisSKD::set_destination(float dist) {
    target_position = dist;
}

float SChassisSKD::present_position() {
    return SChassisIF::present_position();
}

float SChassisSKD::present_velocity() {
    return SChassisIF::present_velocity();
}


void SChassisSKD::SKDThread::main() {
    setName("SChassis_SKD");
    while (!shouldTerminate()) {

        if (mode == ABS_DEST_MODE) {

            target_velocity = a2v_pid.calc(SChassisIF::present_position(), target_position);

            target_current[MOTOR_LEFT] = (int) (v2i_pid[MOTOR_LEFT].calc(
                    SChassisIF::feedback[MOTOR_LEFT].present_velocity * motor_install_[MOTOR_LEFT], target_velocity));

            target_current[MOTOR_RIGHT] = (int) (v2i_pid[MOTOR_RIGHT].calc(
                    SChassisIF::feedback[MOTOR_RIGHT].present_velocity * motor_install_[MOTOR_RIGHT], target_velocity));

        } else if (mode == FORCED_RELAX_MODE) {

            target_current[MOTOR_LEFT] = target_current[MOTOR_RIGHT] = 0;

        }

        // Send currents
        SChassisIF::target_current[MOTOR_LEFT] = target_current[MOTOR_LEFT] * motor_install_[MOTOR_LEFT];
        SChassisIF::target_current[MOTOR_RIGHT] = target_current[MOTOR_RIGHT] * motor_install_[MOTOR_RIGHT];
        SChassisIF::send_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}