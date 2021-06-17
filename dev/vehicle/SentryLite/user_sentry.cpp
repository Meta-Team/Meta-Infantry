//
// Created by Zhanpeng on 2021/1/29.
//

#include "user_sentry.h"
UserS::UserThread UserS::userThread;

void UserS::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}

void UserS::UserThread::main() {
    float YAW_target = SGimbalIF::feedback[SGimbalIF::YAW]->accumulated_angle();
    float PITCH_target = 0;

    while(!shouldTerminate()){
        if(Remote::rc.s1 == Remote::S_MIDDLE) {
            if (SChassisSKD::mode != SChassisSKD::NORMAL_MODE) {
                SChassisSKD::set_mode(SChassisSKD::NORMAL_MODE);
            }
            if (SGimbalSKD::get_mode() != SGimbalSKD::SENTRY_MODE) {
                SGimbalSKD::set_mode(SGimbalSKD::SENTRY_MODE);
            }
            if(Remote::rc.ch1 > 0.5) {
                LED::led_on(6);
            } else {
                LED::led_off(6);
            }
            PITCH_target += Remote::rc.ch3 * 5.0f;
            VAL_CROP(PITCH_target, 0,90);
            YAW_target += Remote::rc.ch2 *0.001f*45.0f;
            VAL_CROP(YAW_target, 90,-90);
            float targetSpeed = Remote::rc.ch1 * 1000;
            SChassisSKD::set_target(targetSpeed);
            SGimbalSKD::set_target_angle(YAW_target, PITCH_target);

            if(Remote::rc.wheel > 0.5) {
                SGimbalSKD::shoot();
            } else {
                SGimbalSKD::stop_shoot();
            }
        } else if (Remote::rc.s1 == Remote::S_DOWN) {
            if (SChassisSKD::mode != SChassisSKD::GROUND_MODE) {
                SChassisSKD::set_mode(SChassisSKD::GROUND_MODE);
            }
            if(Remote::rc.ch0 > 0.5) {
                LED::led_on(7);
            } else {
                LED::led_off(7);
            }
            float targetSpeed = Remote::rc.ch1 * 1000;
            SChassisSKD::set_target(targetSpeed);
            SChassisSKD::turnR = -((float)Remote::rc.ch0)/3.0+0.66;
            SChassisSKD::turnL = +((float)Remote::rc.ch0)/3.0+0.66;
        } else {
            if (SChassisSKD::mode != SChassisSKD::FORCED_RELAX_MODE) {
                SChassisSKD::set_mode(SChassisSKD::FORCED_RELAX_MODE);
            }
            if (SGimbalSKD::get_mode() != SGimbalSKD::FORCED_RELAX_MODE) {
                SGimbalSKD::set_mode(SGimbalSKD::FORCED_RELAX_MODE);
            }
        }

        sleep(TIME_I2MS(5));
    }
}