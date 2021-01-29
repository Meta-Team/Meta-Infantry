//
// Created by Zhanpeng on 2021/1/29.
//

#include "SchassisIF.h"

CANInterface *SchassisIF::can1_ = nullptr;
CANInterface *SchassisIF::can2_ = nullptr;

CANInterface::motor_feedback_t *SchassisIF::feedback[SchassisIF::MOTOR_COUNT];
int *SchassisIF::target_current[SchassisIF::MOTOR_COUNT];

bool SchassisIF::enable_chassis_current_clip() {

    if (!can1_ && !can2_) return false;

#if CHASSIS_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[i], CHASSIS_INTERFACE_MAX_CURRENT);
#endif

    return true;

}

void SchassisIF::init(CANInterface *can1_interface, CANInterface *can2_interface,
                     motor_can_config_t motor_can_config[MOTOR_COUNT]) {
    can1_ = can1_interface;
    can2_ = can2_interface;
    for (int i = 0; i < MOTOR_COUNT; i++) {

        // Check if it's a valid can motor id (0<=id<=7), has motor or use valid can channel.
        // Unsigned < 0 is always false, so no need to judge.
        if (motor_can_config[i].motor_can_id > 7 ||
            motor_can_config[i].motor_type == CANInterface::NONE_MOTOR ||
            motor_can_config[i].motor_can_channel == none_can_channel) {

            // Make the pointer to the "deadzone", which could be easily judged that the motor is invalid.
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
    }
}
