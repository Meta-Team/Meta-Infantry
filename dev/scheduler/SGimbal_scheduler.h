//
// Created by Youch on 2021/6/1.
//

#ifndef META_INFANTRY_SGIMBAL_SCHEDULER_H
#define META_INFANTRY_SGIMBAL_SCHEDULER_H

#include "ch.hpp"
#include "SGimbalIF.h"

#include "pid_controller.hpp"


class SGimbalSKD : public SGimbalBase, public PIDControllerBase{
public:

    enum mode_t {
        FORCED_RELAX_MODE,   // zero force (but still taking control of GimbalIF)
        SENTRY_MODE,         //
    }; // no support for RELATIVE_ANGLE_MODE

    enum install_direction_t {
        POSITIVE = 1,
        NEGATIVE = -1
    };

    static void
    start(install_direction_t yaw_install_, install_direction_t pitch_install_, tprio_t thread_prio);

    static void load_pid_params(pid_params_t yaw_a2v_params, pid_params_t yaw_v2i_params,
                                pid_params_t pitch_a2v_params, pid_params_t pitch_v2i_params);

    static void set_mode(mode_t skd_mode);

    static mode_t get_mode();

    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);

    static float get_target_angle(motor_id_t motor);

    static void shoot();

    static void stop_shoot();

    static void load_shoot_pid_params(PIDController::pid_params_t loader_param, PIDController::pid_params_t shoot_params);

private:
    static install_direction_t yaw_install;
    static install_direction_t pitch_install;

    // Local storage
    static mode_t mode;
    static float target_angle[2];  // angles of MOTORS (NOTICE: different from target_angle in set_target_angle())

    static float last_angle[2];  // last angle data of yaw and pitch from AHRS
    static float accumulated_angle[2];  // accumulated angle of yaw and pitch motors, since the start of this SKD

    static float target_velocity[3];  // calculated target velocity, middle values
    static int target_current[2];     // local storage

    static PIDController a2v_pid[2];
    static PIDController v2i_pid[5];

    static constexpr unsigned int SKD_THREAD_INTERVAL = 1; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;
};


#endif //META_INFANTRY_SGIMBAL_SCHEDULER_H
