//
// Created by Qian Chen on 4/11/21.
//

//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "OLED/oled_trans.h"
#include "debug/shell/shell.h"
// Other headers here

using namespace chibios_rt;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_template(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "template");
        return;
    }
    chprintf(chp, "template" SHELL_NEWLINE_STR);
}


// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"template", cmd_template},
        {nullptr,    nullptr}
};


// Thread to ...
class SkywalkerAdjustThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("template");
        while (!shouldTerminate()) {

            sleep(TIME_MS2I(100));
        }
    }
} templateThread;


int main(void) {
    halInit();
    System::init();
    oled_trans::start(NORMALPRIO);
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    templateThread.start(NORMALPRIO + 1);


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}


