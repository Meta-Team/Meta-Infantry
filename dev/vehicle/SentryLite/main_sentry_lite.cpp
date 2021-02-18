//
// Created by Zhanpeng on 2021/1/29.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"
//
#include "shell.h"
#include "can_interface.h"
#include "remote_interpreter.h"
#include "SChassisSKD.h"
#include "SchassisIF.h"

#include "user_sentry.h"
#include "vehicle_sentry_lite.h"

#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

static SchassisIF::motor_can_config_t CHASSIS_MOTOR_CONFIG_[SchassisIF::MOTOR_COUNT] = CHASSIS_MOTOR_CONFIG;

int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    // Enable power of ultraviolet lights
    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER2_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/
    LED::all_off();
    Shell::start(LOWPRIO+5);
    chThdSleepMilliseconds(50);  // wait for logo to print :)
    BuzzerSKD::init(LOWPRIO);

    LED::led_on(1);
    Remote::start();
    can1.start(HIGHPRIO, HIGHPRIO-1);
    can2.start(HIGHPRIO-2,HIGHPRIO-3);
    chThdSleepMilliseconds(5);
    UserS::start(NORMALPRIO + 1);
    chThdSleepMilliseconds(5);
    LED::led_on(2);
    SchassisIF::init(&can1, &can2, CHASSIS_MOTOR_CONFIG_);
    chThdSleepMilliseconds(5);
    LED::led_on(3);
    SChassisSKD::start(1, 1, 1, NORMALPRIO);
    SChassisSKD::load_pid_params(CHASSIS_PID_V2I_PARAMS, CHASSIS_PID_V2I_PARAMS);
    chThdSleepMilliseconds(5);
    LED::led_on(4);
    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);  // Now play the startup sound
    chThdSleepMilliseconds(5);
    LED::led_on(5);
    chThdSleepMilliseconds(5);

    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}