//
// Created by Youch on 2021/6/1.
//

#include "SGimbal_scheduler.h"

SGimbalSKD::mode_t SGimbalSKD::mode = FORCED_RELAX_MODE;
SGimbalSKD::install_direction_t SGimbalSKD::yaw_install;
SGimbalSKD::install_direction_t SGimbalSKD::pitch_install;


float SGimbalSKD::target_angle[2] = {0, 0};
float SGimbalSKD::target_velocity[3] = {0, 0};
int SGimbalSKD::target_current[2] = {0, 0};
float SGimbalSKD::last_angle[2] = {0, 0};
float SGimbalSKD::accumulated_angle[2] = {0, 0};
PIDController SGimbalSKD::a2v_pid[2];
PIDController SGimbalSKD::v2i_pid[5];
SGimbalSKD::SKDThread SGimbalSKD::skdThread;

void
SGimbalSKD::start(install_direction_t yaw_install_, install_direction_t pitch_install_, tprio_t thread_prio) {


    yaw_install = yaw_install_;
    pitch_install = pitch_install_;


    // Initialize last_angle, to use current pointing direction as startup direction


    // FIXME: find a more elegant way to handle this
#if defined(SENTRY) || defined(AERIAL)
    last_angle[YAW] = GimbalIF::feedback[GimbalIF::YAW]->actual_angle * yaw_install;
#else
    last_angle[YAW] = SGimbalIF::feedback[SGimbalIF::YAW]->actual_angle * (float)yaw_install;
    // For initial moment, angle_movement = ahrs_angle.x - last_angle[YAW] = GimbalIF::feedback[YAW].actual_angle
#endif

    last_angle[PITCH] = SGimbalIF::feedback[SGimbalIF::PITCH]->actual_angle * (float)pitch_install;

    skdThread.start(thread_prio);
}

void SGimbalSKD::load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params) {
    a2v_pid[YAW].change_parameters(yaw_a2v_params);
    v2i_pid[YAW].change_parameters(yaw_v2i_params);

    a2v_pid[PITCH].change_parameters(pitch_a2v_params);
    v2i_pid[PITCH].change_parameters(pitch_v2i_params);
}

void SGimbalSKD::set_mode(SGimbalSKD::mode_t skd_mode) {
    mode = skd_mode;
}

SGimbalSKD::mode_t SGimbalSKD::get_mode() {
    return mode;
}

void SGimbalSKD::set_target_angle(float yaw_target_angle, float pitch_target_angle) {
    target_angle[YAW] = yaw_target_angle;
    target_angle[PITCH] = pitch_target_angle;
}

float SGimbalSKD::get_target_angle(SGimbalBase::motor_id_t motor) {
    return target_angle[motor];
}

void SGimbalSKD::shoot() {
    target_velocity[2] = 300.0f;
}

void SGimbalSKD::stop_shoot() {
    target_velocity[2] = 0.0f;
}

void SGimbalSKD::load_shoot_pid_params(PIDController::pid_params_t loader_param, PIDController::pid_params_t shoot_params) {
    v2i_pid[2].change_parameters(loader_param);
    v2i_pid[3].change_parameters(shoot_params);
    v2i_pid[4].change_parameters(shoot_params);
}

void SGimbalSKD::SKDThread::main() {
    setName("GimbalSKD");
    while (!shouldTerminate()) {

        // Fetch data


        // TODO: document calculations here
        float angle[2] = {SGimbalIF::feedback[SGimbalSKD::YAW]->accumulated_angle(), SGimbalIF::feedback[SGimbalIF::PITCH]->accumulated_angle()};
        float velocity[2] = {SGimbalIF::feedback[SGimbalIF::PITCH]->actual_velocity, SGimbalIF::feedback[SGimbalIF::PITCH]->actual_velocity};

        for (int i = YAW; i <= PITCH; i++) {

            float angle_movement = angle[i] - last_angle[i];
            last_angle[i] = angle[i];

            /**
             * Deal with cases crossing 0 point
             * For example,
             * 1. new = 179, last = -179, movement = 358, should be corrected to 358 - 360 = -2
             * 2. new = -179, last = 179, movement = -358, should be corrected to -358 + 360 = 2
             * 200 (-200) is a threshold that is large enough that it's normally impossible to move in 1 ms
             */
            if (angle_movement < -200) angle_movement += 360;
            if (angle_movement > 200) angle_movement -= 360;

            // Use increment to calculate accumulated angles
            accumulated_angle[i] += angle_movement;
        }

        if (mode == SENTRY_MODE) {

            /// Yaw
            // Use gimbal motor feedback angle and velocity
            target_velocity[YAW] = a2v_pid[YAW].calc(
                    SGimbalIF::feedback[YAW]->accumulated_angle(),
                    target_angle[YAW]);
            target_current[YAW] = (int) v2i_pid[YAW].calc(
                    SGimbalIF::feedback[YAW]->actual_velocity,
                    target_velocity[YAW]);

            /// Pitch
            // Use gimbal motor feedback angle and AHRS velocity
            target_velocity[PITCH] = a2v_pid[PITCH].calc(
                    SGimbalIF::feedback[PITCH]->accumulated_angle(),
                    target_angle[PITCH]);
            target_current[PITCH] = (int) v2i_pid[PITCH].calc(velocity[PITCH], target_velocity[PITCH]);

            /// Shoot
            *SGimbalIF::target_current[FW_LEFT] = (int)(v2i_pid[FW_LEFT].calc(SGimbalIF::feedback[FW_LEFT]->actual_velocity, 2000));
            *SGimbalIF::target_current[FW_RIGHT] = (int)(v2i_pid[FW_RIGHT].calc(SGimbalIF::feedback[FW_RIGHT]->actual_velocity, -2000));
            *SGimbalIF::target_current[BULLET] = (int)(v2i_pid[BULLET].calc(SGimbalIF::feedback[BULLET]->actual_velocity, target_velocity[2]));
            *SGimbalIF::target_current[PITCH] = target_current[PITCH];
            *SGimbalIF::target_current[YAW] = target_current[YAW];

        } else if (mode == FORCED_RELAX_MODE) {

            target_current[YAW] = target_current[PITCH] = target_current[BULLET] = 0;
            *SGimbalIF::target_current[FW_LEFT] = (int)(v2i_pid[FW_LEFT].calc(SGimbalIF::feedback[FW_LEFT]->actual_velocity, 0));
            *SGimbalIF::target_current[FW_RIGHT] = (int)(v2i_pid[FW_RIGHT].calc(SGimbalIF::feedback[FW_RIGHT]->actual_velocity, 0));
            *SGimbalIF::target_current[BULLET] = target_current[BULLET];
            *SGimbalIF::target_current[PITCH] = target_current[PITCH];
            *SGimbalIF::target_current[YAW] = target_current[YAW];
        }

        // Send currents
        *SGimbalIF::target_current[YAW] = target_current[YAW];
        *SGimbalIF::target_current[PITCH] = target_current[PITCH];
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}