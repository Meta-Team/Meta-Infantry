//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    shoot_logic.cpp
 * @brief   Logic-level module to control shooter. Support number-controlling shooting.
 *
 * @addtogroup shoot
 * @{
 */


#include "shoot_logic.h"

#include "shell.h"
#include "shoot_scheduler.h"
#include "referee_interface.h"

float ShootLG::angle_per_bullet = 0;
int ShootLG::bullet_count = 0;
float ShootLG::shoot_target_number = 0;
ShootLG::shooter_state_t ShootLG::shooter_state = STOP;
ShootLG::StuckDetectorThread ShootLG::stuckDetector;
chibios_rt::ThreadReference ShootLG::stuckDetectorReference;

ShootLG::BulletCounterThread ShootLG::bulletCounterThread;

void ShootLG::init(float angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t bullet_counter_thread_prio) {
    angle_per_bullet = angle_per_bullet_;
    stuckDetectorReference = stuckDetector.start(stuck_detector_thread_prio);
    bulletCounterThread.start(bullet_counter_thread_prio);
}

void ShootLG::increment_bullet(int number_of_bullet) {
    bullet_count += number_of_bullet;
}

void ShootLG::set_bullet_count(int number_of_bullet) {
    bullet_count = number_of_bullet;
}

int ShootLG::get_bullet_count() {
    return bullet_count;
}

void ShootLG::set_friction_wheels(float duty_cycle) {
    ShootSKD::set_friction_wheels(duty_cycle);
    Referee::set_client_light(USER_CLIENT_FW_STATE_LIGHT, (duty_cycle != 0));
#if defined(INFANTRY)
    // TODO: re-arrange here for common vehicle
    Referee::set_client_number(USER_CLIENT_FW_SPEED_NUM, duty_cycle);
#endif
    // Sending client data will be complete by higher level thread
}

float ShootLG::get_friction_wheels_duty_cycle() {
    return ShootSKD::get_friction_wheels_duty_cycle();
}

ShootLG::shooter_state_t ShootLG::get_shooter_state() {
    return shooter_state;
}

void ShootLG::shoot(float number_of_bullet, float number_per_second) {
    shoot_target_number = number_of_bullet;

    shooter_state = SHOOTING;
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);
    ShootSKD::set_loader_target_velocity(number_per_second * angle_per_bullet);
    ShootSKD::set_loader_target_angle(shoot_target_number * angle_per_bullet);
    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    if (!stuckDetector.started) {
        stuckDetector.started = true;
        stuckDetector.waited = false;
        chSchWakeupS(stuckDetectorReference.getInner(), 0);
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}

void ShootLG::stop() {
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_loader_target_angle(0.0f);
    shooter_state = STOP;
}

void ShootLG::force_stop() {
    ShootSKD::reset_loader_accumulated_angle();
    ShootSKD::set_loader_target_angle(0.0f);
    ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
    shooter_state = STOP;
}

void ShootLG::StuckDetectorThread::main() {
    setName("Shoot_Stuck");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (shooter_state != SHOOTING) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if (!waited) {
            sleep(TIME_MS2I(STUCK_DETECTOR_INITIAL_WAIT_INTERVAL));
            waited = true;
        }

//        if (ShootSKD::get_loader_target_current() > STUCK_THRESHOLD_CURRENT &&
//            ShootSKD::get_loader_actual_velocity() < STUCK_THRESHOLD_VELOCITY) {
        if (false){
            shooter_state = STUCK;
            LOG_WARN("Bullet loader stuck");
            ShootSKD::set_loader_target_angle(ShootSKD::get_loader_accumulated_angle() - STUCK_REVERSE_ANGLE);
            sleep(TIME_MS2I(STUCK_REVERSE_TIME));
            if (shooter_state == STOP) {  // if shooter is stopped during the pausing time, terminate the process.
                continue;
            }
            ShootSKD::set_loader_target_angle(shoot_target_number * angle_per_bullet);  // recover original target
            shooter_state = SHOOTING;

        }

        sleep(TIME_MS2I(STUCK_DETECTOR_THREAD_INTERVAL));
    }
}

void ShootLG::BulletCounterThread::main() {
    setName("Shoot_Count");

    chEvtRegisterMask(&Referee::data_received_event, &data_received_listener, DATA_RECEIVED_EVENTMASK);

    while (!shouldTerminate()) {

        chEvtWaitAny(DATA_RECEIVED_EVENTMASK);

        eventflags_t flags = chEvtGetAndClearFlags(&data_received_listener);

        // Toggle Referee LED if any data is received
        LED::led_toggle(DEV_BOARD_LED_REFEREE);

        // Add bullets
        if (flags == Referee::SUPPLY_PROJECTILE_ACTION_CMD_ID &&
            Referee::supply_projectile_action.supply_robot_id == Referee::get_self_id() &&
            Referee::supply_projectile_action.supply_projectile_step == 2  // bullet fall
                ) {
            bullet_count += (int) (Referee::supply_projectile_action.supply_projectile_num * 1.0f);
        }

        Referee::set_client_number(USER_CLIENT_ACQUIRED_BULLET_NUM, bullet_count);
    }
}

/** @} */
