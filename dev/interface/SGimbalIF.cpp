//
// Created by Youch on 2021/6/1.
//

#include "SGimbalIF.h"
CANInterface::motor_feedback_t *SGimbalIF::feedback[MOTOR_COUNT];
int *SGimbalIF::target_current[MOTOR_COUNT];
CANInterface *SGimbalIF::can1_ = nullptr;
CANInterface *SGimbalIF::can2_ = nullptr;

void SGimbalIF::init(CANInterface *can1_interface,                      CANInterface *can2_interface,
                    motor_can_config_t motor_can_config[MOTOR_COUNT],
                    uint16_t yaw_front_angle_raw) {

    // Get the CAN address.
    can1_ = can1_interface;
    can2_ = can2_interface;

    // Link the feedback to feedback info stored in CANInterface;
    // The feedback[i] is the i'th motor to receive feedback and it needs an address of the real motor feedback;
    // The i is between [YAW,FW_RIGHT] and gimbal::init will receive the motor_can_config which contains{can channel, can's id, motor type };
    // There are 6 motors in gimbal, so the can_config array passed in should contains 6 elements;
    // There are two cans being used, so the i'th motor in the gimbal should first judge which can does it belong to:
    //          motor_can_config[i].motor_can_channel == can_channel_1;
    // After knowing which can does the i'th motor belongs to, this motor's feedback array should be linked to the data in the can Interface,
    // because the real feedback data was stored in the processFeedbackThread.feedback[] and the feedback array stored in the gimbal interface was an array of pointers;
    // Then the i'th motor should use the code below to get the address:
    //          can1_->get_feedback_address(motor_can_config[i].motor_can_id);
    // or       can2_->get_feedback_address(motor_can_config[i].motor_can_id);
    // Note the function return:
    //          return &processFeedbackThread.feedback[id];
    // Note that the i'th motor has the i'th motor_can_config value which was loaded on the motor_can_id'th position of the corresponding can wire;
    for (int i = 0; i < MOTOR_COUNT; i++) {

        // Check if it's a valid can motor id (0<=id<=7), has motor or use valid can channel.
        // Unsigned < 0 is always false, so no need to judge.
        if(motor_can_config[i].motor_can_id > 7 ||
           motor_can_config[i].motor_type == CANInterface::NONE_MOTOR ||
           motor_can_config[i].motor_can_channel == none_can_channel) {

            // Make the pointer to the "deadzone", which could indicate that motor is invalid.
            feedback[i] = can1_->get_feedback_address(CANInterface::MAXIMUM_MOTOR_COUNT);
            target_current[i] = can1_->get_target_current_address(CANInterface::MAXIMUM_MOTOR_COUNT);
            continue;
        }

        if (motor_can_config[i].motor_can_channel == can_channel_1) {

            // Link the feedback and target_current to can1's feedback
            can1_->set_motor_type(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            feedback[i] = can1_->get_feedback_address(motor_can_config[i].motor_can_id);
            target_current[i] = can1_->get_target_current_address(motor_can_config[i].motor_can_id);
            *target_current[i] = 0;

        } else if (motor_can_config[i].motor_can_channel == can_channel_2) {

            // Link the feedback and target_current to can2's feedback
            can2_->set_motor_type(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            feedback[i] = can2_->get_feedback_address(motor_can_config[i].motor_can_id);
            target_current[i] = can2_->get_target_current_address(motor_can_config[i].motor_can_id);
            *target_current[i] = 0;

        }

        // Set front angle of yaw and pitch
        if(YAW == (motor_id_t) i) {
            feedback[i]->last_angle_raw = yaw_front_angle_raw;
        }

    }

    chThdSleep(TIME_MS2I(500));

}