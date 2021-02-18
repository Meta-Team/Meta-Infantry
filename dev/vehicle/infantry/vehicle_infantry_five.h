//
// Created by liuzikai on 2019-01-28.
//

#ifndef META_INFANTRY_MAIN_INFANTRY_FIVE_H
#define META_INFANTRY_MAIN_INFANTRY_FIVE_H

#include "vehicle_infantry.h"

/** Installation Based Params **/

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 3337
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 7256  // of no use now

#define MPU6500_STORED_GYRO_BIAS {-0.563549041, -1.174620985, 0.222040429}


#endif //META_INFANTRY_MAIN_INFANTRY_FIVE_H
