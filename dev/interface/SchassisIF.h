//
// Created by Zhanpeng on 2021/1/29.
//

#ifndef META_INFANTRY_SCHASSISIF_H
#define META_INFANTRY_SCHASSISIF_H

#include "interface/can_interface.h"

class SChassisBase {
public:
    // Start at the front right, goes in CCW
    enum motor_id_t {
        FR, // front right motor, 0
        FL, // front left motor, 1
        MOTOR_COUNT
    };
};

class SchassisIF: public SChassisBase {

public:

    enum motor_can_channel_t {
        none_can_channel,
        can_channel_1,
        can_channel_2
    };

    struct motor_can_config_t {
        motor_can_channel_t motor_can_channel;
        unsigned motor_can_id;
        CANInterface::motor_type_t motor_type;
    };

    /**
     * Set CAN interface for receiving and sending
     * @param can1_   Initialized CANInterface for chassis motors
     */
    static void init(CANInterface* can1_interface, CANInterface *can2_interface, motor_can_config_t motor_can_config[MOTOR_COUNT]);

    /** Structure for each motor */

    /**
     * Feedback for each chassis motor
     */
    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];

    /**
     * Target current array in the order defined in motor_id_t
     */
    static int *target_current[MOTOR_COUNT];

    /**
     * Send all target currents
     * @return Whether sending succeeded or not
     */
    static bool enable_chassis_current_clip();

private:

    static CANInterface* can1_;
    static CANInterface* can2_;

    friend CANInterface;

private:

    static float constexpr CHASSIS_MOTOR_DECELERATE_RATIO = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_SCHASSISIF_H
