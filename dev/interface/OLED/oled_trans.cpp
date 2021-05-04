//
// Created by Qian Chen on 4/10/21.
//

#include "oled_trans.h"

#define X_WIDTH 128
#define Y_WIDTH 64
oled_trans::UpdateThread oled_trans::updateThread;


void oled_trans::start(tprio_t updateprio) {

    spiAcquireBus(&OLED_SPI_DRIVER);
    spiSelect(&OLED_SPI_DRIVER);

    spiUnselect(&OLED_SPI_DRIVER);
    spiReleaseBus(&OLED_SPI_DRIVER);

    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;

    updateThread.start(updateprio);
}

void oled_write_byte(uint8_t dat)
{
    spiSend(&OLED_SPI_DRIVER,1,&dat);    //display on
}

static void oled_set_pos(uint8_t x, uint8_t y)
{
    x += 2;
    oled_write_byte((0xb0 + y));              //set page address y
    oled_write_byte(((x&0xf0)>>4)|0x10);      //set column high address
    oled_write_byte((x&0x0f));                //set column low address
}



void oled_trans::UpdateThread::main() {
    setName("oled update thread");
    while (!shouldTerminate()){
        spiAcquireBus(&OLED_SPI_DRIVER);
        spiSelect(&OLED_SPI_DRIVER);

        spiUnselect(&OLED_SPI_DRIVER);
        spiReleaseBus(&OLED_SPI_DRIVER);


        sleep(TIME_MS2I(500));
    }
}