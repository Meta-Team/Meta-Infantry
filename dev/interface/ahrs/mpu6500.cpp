#include "mpu6500.h"

#include "led.h"
#include "shell.h"
#include "math.h"

static const SPIConfig SPI5_cfg =
        {
                false,
                nullptr,
                MPU6500_SPI_CS_PAD,
                MPU6500_SPI_CS_PIN,
                SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR |
                SPI_CR1_CPHA | SPI_CR1_CPOL, //Set CPHA and CPOL to be 1
                0
        };

/**
 * @note SPI Read/Write Protocol
 * 1. (ChibiOS) Lock the SPI driver (spiAcquireBus)
 * 2. CS (Chip Select) down (spiSelect)
 * 3. Send register address N (bit 7 is 0 for write, 1 for read)
 * 4. Read/Write register N
 * (Optional, repeated) Read/Write register N+1, N+2, N+3, ...
 * 5. CS up (spiUnselect)
 * 6. (ChibiOS) unlock the SPI driver (spiReleaseBus)
 */

/**
 * Write a single register through SPI.
 * @param reg_addr The register
 * @param value    The value to write
 */
void mpu6500_write_reg(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {reg_addr /* bit 7 is keep as 0 for write */, value};
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 2, tx_data);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);
}

void MPUOnBoard::start(tprio_t prio) {

    // Start SPI driver
    spiStart(&MPU6500_SPI_DRIVER, &SPI5_cfg);

    // Reset MPU6500
    mpu6500_write_reg(MPU6500_PWR_MGMT_1, MPU6500_RESET);
    chThdSleepMilliseconds(100);  // wait for MPU6500 to reset, see data sheet

    // Enable MPU6500 as I2C Master (for IST8310) and disable as I2C Slave
    // MPU6500_I2C_IF_DIS disable MPU6500 as I2C Slave and enable SPI only, by datasheet p.30 6.1 Note
    mpu6500_write_reg(MPU6500_USER_CTRL, MPU6500_I2C_MST_EN | MPU6500_I2C_IF_DIS);

    // Configure MPU6500 for gyro and accel
    uint8_t init_reg[5][2] = {
            {MPU6500_PWR_MGMT_1, MPU6500_AUTO_SELECT_CLK},  // auto clock
            {MPU6500_CONFIG,         CONFIG._dlpf_config},
            {MPU6500_GYRO_CONFIG,    CONFIG._gyro_scale << 3U /* [1:0] for FCHOICE_B = b00, low-pass-filter enabled */},
            {MPU6500_ACCEL_CONFIG,   CONFIG._accel_scale << 3U},
            {MPU6500_ACCEL_CONFIG_2, CONFIG._acc_dlpf_config}
    };
    for (auto & i : init_reg) {
        mpu6500_write_reg(i[0], i[1]);
        chThdSleepMilliseconds(10);
    }

    temperature = 0;

    // Get the coefficient converting the raw data to degree or gravity
    switch (CONFIG._gyro_scale) {
        case MPU6500_GYRO_SCALE_250:
            gyro_psc = (1.0f / 131.0f);
            break;
        case MPU6500_GYRO_SCALE_500:
            gyro_psc = (1.0f / 65.5f);
            break;
        case MPU6500_GYRO_SCALE_1000:
            gyro_psc = (1.0f / 32.8f);
            break;
        case MPU6500_GYRO_SCALE_2000:
            gyro_psc = (1.0f / 16.4f);
            break;
        default:
            gyro_psc = 0.0f;
            break;
    }

    switch (CONFIG._accel_scale) {
        case MPU6500_ACCEL_SCALE_2G:
            accel_psc = (1 / 16384.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_4G:
            accel_psc = (1 / 8192.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_8G:
            accel_psc = (1 / 4096.0f) * GRAV_CONSTANT;
            break;
        case MPU6500_ACCEL_SCALE_16G:
            accel_psc = (1 / 2048.0f) * GRAV_CONSTANT;
            break;
        default:
            accel_psc = 0.0f;
            break;
    }

#if MPU6500_ENABLE_ACCEL_BIAS
    // Set initial value for no rotation, used in updateThread
    accel_rotation[0][0] = 1.0f;
    accel_rotation[0][1] = 0.0f;
    accel_rotation[0][2] = 0.0f;
    accel_rotation[1][0] = 0.0f;
    accel_rotation[1][1] = 1.0f;
    accel_rotation[1][2] = 0.0f;
    accel_rotation[2][0] = 0.0f;
    accel_rotation[2][1] = 0.0f;
    accel_rotation[2][2] = 1.0f;
#endif

    // Start the update thread
    updateThread.start(prio);

    // Wait for first calibration
    while (last_calibration_time == 0) {
        chThdSleepMilliseconds(1);
    }
}

void MPUOnBoard::load_calibration_data(Vector3D gyro_bias_) {
    gyro_bias = gyro_bias_;
    last_calibration_time = SYSTIME;
}

bool MPUOnBoard::MPU6500ready() {
    return MPU6500_startup_calibrated;
}

void MPUOnBoard::update() {

    // Fetch data from SPI
    uint8_t tx_data = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
    spiAcquireBus(&MPU6500_SPI_DRIVER);
    spiSelect(&MPU6500_SPI_DRIVER);
    spiSend(&MPU6500_SPI_DRIVER, 1, &tx_data);
//    chThdSleepMilliseconds(1);
    spiReceive(&MPU6500_SPI_DRIVER, RX_BUF_SIZE, rx_buf);
    spiUnselect(&MPU6500_SPI_DRIVER);
    spiReleaseBus(&MPU6500_SPI_DRIVER);

    chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
    {

        /// Decode data

        accel_raw = Vector3D((int16_t) (rx_buf[0] << 8 | rx_buf[1]),
                             (int16_t) (rx_buf[2] << 8 | rx_buf[3]),
                             (int16_t) (rx_buf[4] << 8 | rx_buf[5])) * accel_psc;

        Vector3D new_gyro_orig = Vector3D((int16_t) (rx_buf[8] << 8 | rx_buf[9]),
                                          (int16_t) (rx_buf[10] << 8 | rx_buf[11]),
                                          (int16_t) (rx_buf[12] << 8 | rx_buf[13])) * gyro_psc;

        temperature = ((((int16_t) (rx_buf[6] << 8 | rx_buf[7])) - TEMPERATURE_BIAS) / 333.87f) + 21.0f;

        /// Gyro Calibration sampling

        if ((ABS_IN_RANGE(new_gyro_orig.x - gyro_raw.x, STATIC_RANGE) &&
             ABS_IN_RANGE(new_gyro_orig.y - gyro_raw.y, STATIC_RANGE) &&
             ABS_IN_RANGE(new_gyro_orig.z - gyro_raw.z, STATIC_RANGE)) &&
            !MPU6500_startup_calibrated) {  // MPU6500 static

            static_measurement_count++;
            temp_gyro_bias = temp_gyro_bias - new_gyro_orig;  // bias is to be minus, so sum as negative here

#if MPU6500_ENABLE_ACCEL_BIAS
            temp_accel_bias = temp_accel_bias + accel;
#endif
        }

        /// Bias data

        gyro_raw = new_gyro_orig;
        gyro = gyro_raw + gyro_bias;
#if MPU6500_ENABLE_ACCEL_BIAS
        accel = accel_orig * accel_rotation;
#else
        accel = accel_raw;
#endif

        /// Update info

        mpu_update_time = SYSTIME;


        /// Perform gyro re-bias

        if (!MPU6500_startup_calibrated && (static_measurement_count >= BIAS_SAMPLE_COUNT) &&
            (SYSTIME - updateThread.thread_start_time > mpu6500_start_up_time)) {
//        chSysUnlock();  /// --- EXIT S-Locked state ---
            MPU6500_startup_calibrated = true;
            gyro_bias = temp_gyro_bias / static_measurement_count;
#if MPU6500_ENABLE_ACCEL_BIAS
            if (last_calibration_time == 0) {
                    // Only calibrate accel for ones
                    accel_rotation[2][0] = temp_accel_bias.x / BIAS_SAMPLE_COUNT;
                    accel_rotation[2][1] = temp_accel_bias.y / BIAS_SAMPLE_COUNT;
                    accel_rotation[2][2] = temp_accel_bias.z / BIAS_SAMPLE_COUNT;
                    accel_rotation[0][0] = accel_rotation[2][1] - accel_rotation[2][2];
                    accel_rotation[0][1] = accel_rotation[2][2] - accel_rotation[2][0];
                    accel_rotation[0][0] = accel_rotation[2][0] - accel_rotation[2][1];
                    float length = sqrtf(accel_rotation[0][0] * accel_rotation[0][0] + accel_rotation[0][1] * accel_rotation[0][1]
                                         + accel_rotation[0][2] * accel_rotation[0][2]);
                    accel_rotation[0][0] /= length;
                    accel_rotation[0][1] /= length;
                    accel_rotation[0][2] /= length;
                    Vector3D temp_vect = Vector3D(accel_rotation[0]).crossMultiply(Vector3D(accel_rotation[2]));
                    accel_rotation[1][0] = temp_vect.x;
                    accel_rotation[1][1] = temp_vect.y;
                    accel_rotation[1][2] = temp_vect.z;
                }
#endif
            static_measurement_count = 0;
            temp_gyro_bias = Vector3D(0, 0, 0);
            last_calibration_time = SYSTIME;
        }
    }
    chSysUnlock();  /// --- EXIT S-Locked state ---
}


void MPUOnBoard::UpdateThread::main() {
    setName("MPU6500");
    thread_start_time = SYSTIME;
    imu.MPU6500_startup_calibrated = false;
    while (!shouldTerminate()) {

        imu.update();

        sleep(TIME_MS2I(THREAD_UPDATE_INTERVAL));
    }
}