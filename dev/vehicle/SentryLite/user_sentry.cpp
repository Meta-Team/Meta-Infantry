//
// Created by Zhanpeng on 2021/1/29.
//

#include "user_sentry.h"
UserS::UserThread UserS::userThread;

void UserS::start(tprio_t user_thd_prio) {
    userThread.start(user_thd_prio);
}

void UserS::UserThread::main() {
    while(!shouldTerminate()){
        if(Remote::rc.s1 == Remote::S_MIDDLE) {
            if (SChassisSKD::mode != SChassisSKD::NORMAL_MODE) {
                SChassisSKD::set_mode(SChassisSKD::NORMAL_MODE);
            }
            if(Remote::rc.ch1 > 0.5) {
                LED::led_on(6);
            } else {
                LED::led_off(6);
            }
            float targetSpeed = Remote::rc.ch1 * 1000;
            SChassisSKD::set_target(targetSpeed);
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
        }

        sleep(TIME_I2MS(5));
    }
}