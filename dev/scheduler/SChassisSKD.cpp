//
// Created by Zhanpeng on 2021/1/29.
//

#include "SChassisSKD.h"

SChassisSKD::mode_t SChassisSKD::mode = FORCED_RELAX_MODE;

float SChassisSKD::target_vx;

PIDController SChassisSKD::a2v_pid;
PIDController SChassisSKD::v2i_pid[MOTOR_COUNT];

float SChassisSKD::target_velocity[MOTOR_COUNT];
SChassisSKD::SKDThread SChassisSKD::skdThread;
int SChassisSKD::target_current[MOTOR_COUNT];
float SChassisSKD::w_to_v_ratio_;
float SChassisSKD::turnL;
float SChassisSKD::turnR;

void SChassisSKD::start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio) {
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 180.0f * 3.14159f;
    skdThread.start(thread_prio);
}

void SChassisSKD::load_pid_params(PIDControllerBase::pid_params_t theta2v_pid_params,
                                 PIDControllerBase::pid_params_t v2i_pid_params) {
    a2v_pid.change_parameters(theta2v_pid_params);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        v2i_pid[i].change_parameters(v2i_pid_params);
    }
}

void SChassisSKD::set_mode(SChassisSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void SChassisSKD::set_target(float vx) {
    target_vx = vx;
}

void SChassisSKD::SKDThread::main() {
    setName("SChassis_SKD");
    while (!shouldTerminate()) {
        if(mode == FORCED_RELAX_MODE) {
            target_current[1] = target_current[0] = 0;
        } else if (mode == NORMAL_MODE){
            target_velocity[0] = -target_vx;
            target_velocity[1] = target_vx;
            for (int i = 0; i<MOTOR_COUNT; i++){
                target_current[i] = (int)v2i_pid[i].calc(SchassisIF::feedback[i]->actual_velocity, target_velocity[i]);
            }
        } else if (mode == GROUND_MODE) {
            target_velocity[0] = -target_vx* turnL;
            target_velocity[1] = target_vx*turnR;
            for (int i = 0; i<MOTOR_COUNT; i++){
                target_current[i] = (int)v2i_pid[i].calc(SchassisIF::feedback[i]->actual_velocity, target_velocity[i]);
            }
        }
        for(int i = 0; i<MOTOR_COUNT; i++) {
            *SchassisIF::target_current[i] = target_current[i];
        }
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}
