//
// Created by Zhanpeng on 2021/1/29.
//

#ifndef META_INFANTRY_SCHASSISSKD_H
#define META_INFANTRY_SCHASSISSKD_H

#include "SchassisIF.h"
#include "ch.hpp"
#include "pid_controller.hpp"
#include "math.h"

class SChassisSKD : public SChassisBase, PIDControllerBase{
public:
    enum mode_t {
        FORCED_RELAX_MODE,         // zero force (Still taking control of ChassisIF. External writing to target currents
        // will leads to conflicts.)
        // NO SUPPORT FOR CHASSIS COORDINATE
        NORMAL_MODE
    };

    /**
     * Initialize ChassisInterface and this calculator
     * @param wheel_base              Distance between front axle and the back axle [mm]
     * @param wheel_tread             Distance between left and right wheels [mm]
     * @param wheel_circumference     Circumference of wheels [mm]
     * @param install_mode            Whether chassis motors are reversed with some mechanism
     * @param chassis_gimbal_offset   Distance between gimbal and chassis [mm, + for gimbal at "front"]
     * @param thread_prio             Priority of PID calculation thread
     */
    static void start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio);
    /**
     * Change PID parameters of PID controller
     * @param theta2v_pid_params   Theta (see set_target()) to chassis rotation PID parameters
     * @param v2i_pid_params       Velocity to current parameters of every motor (shared parameters)
     */
    static void load_pid_params(pid_params_t theta2v_pid_params, pid_params_t v2i_pid_params);

    /**
     * Set mode of this SKD
     * @param skd_mode
     */
    static void set_mode(mode_t skd_mode);
    /**
     * Set target values
     * @param vx     Target velocity along the x axis (right) with respect to gimbal coordinate [mm/s]
     */
    static void set_target(float vx);

private:
    static mode_t mode;
    static float target_vx;
    static PIDController a2v_pid;               // for theta control
    static PIDController v2i_pid[MOTOR_COUNT];  // speed control for each motor
    static float target_velocity[MOTOR_COUNT];
    static int target_current[MOTOR_COUNT];     // local storage of target current of each motor
    static float w_to_v_ratio_;

    static constexpr unsigned int SKD_THREAD_INTERVAL = 2; // PID calculation interval [ms]

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static SKDThread skdThread;
};

#endif //META_INFANTRY_SCHASSISSKD_H
