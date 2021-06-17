//
// Created by Zhanpeng on 2021/1/29.
//

#ifndef META_INFANTRY_USER_SENTRY_H
#define META_INFANTRY_USER_SENTRY_H

#include "remote_interpreter.h"
#include "SChassisSKD.h"
#include "SGimbal_scheduler.h"

class UserS {
public:
    static void start(tprio_t user_thd_prio);
private:
    class UserThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };
    static UserThread userThread;
};

#endif //META_INFANTRY_USER_SENTRY_H
