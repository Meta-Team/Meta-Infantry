//
// Created by Youch on 2021/6/1.
//

#ifndef META_INFANTRY_SGIMBALIF_H
#define META_INFANTRY_SGIMBALIF_H
#include "can_interface.h"

class SGimbalBase {
public:
    enum motor_id_t{
        YAW,
        PITCH,
        BULLET,
        FW_LEFT,
        FW_RIGHT,
        MOTOR_COUNT
    };
};

class SGimbalIF: public SGimbalBase {
public:
    enum motor_can_channel_t{
        none_can_channel,
        can_channel_1,
        can_channel_2,
    };

    struct motor_can_config_t{
        motor_can_channel_t motor_can_channel;
        unsigned  motor_can_id;
        CANInterface::motor_type_t motor_type;
    };

    static void init(CANInterface* can1_interface,  CANInterface* can2_interface,
                     motor_can_config_t motor_can_config[MOTOR_COUNT],
                     uint16_t yaw_front_angle_raw);

    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];

    static int *target_current[MOTOR_COUNT];




private:
    static CANInterface* can1_;
    static CANInterface* can2_;
};


#endif //META_INFANTRY_SGIMBALIF_H
