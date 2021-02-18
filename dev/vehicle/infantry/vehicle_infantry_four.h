//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FOUR_H
#define META_INFANTRY_MAIN_INFANTRY_FOUR_H

#include "vehicle_infantry.h"

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 6115
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 2033  // of no use now

#define MPU6500_STORED_GYRO_BIAS {1.111804723f, -0.881306171f, 0.140071436f}

#endif //META_INFANTRY_MAIN_INFANTRY_FOUR_H
