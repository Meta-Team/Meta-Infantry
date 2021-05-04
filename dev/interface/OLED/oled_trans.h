//
// Created by Qian Chen on 4/10/21.
//

#ifndef META_INFANTRY_OLED_TRANS_H
#define META_INFANTRY_OLED_TRANS_H

#define OLED_SPI_DRIVER SPID1

#include "ch.hpp"
#include "hal.h"
#include "common_macro.h"

class oled_trans {
public:
    static void start(tprio_t updateprio);
private:

    class UpdateThread: public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static UpdateThread updateThread;
};


#endif //META_INFANTRY_OLED_TRANS_H
