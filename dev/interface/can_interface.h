//
// Created by liuzikai on 2018-12-29.
//

/**
 * @file    can_interface.h
 * @brief   CAN interface to receive, distribute and send message.
 *
 * @addtogroup CANInterface
 * @{
 */

#ifndef META_INFANTRY_CAN_INTERFACE_H
#define META_INFANTRY_CAN_INTERFACE_H

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#if defined(BOARD_RM_2018_A)
// CAN1_RX - PD0, CAN1_TX - PD1
#elif defined(BOARD_RM_2017)
// CAN1_RX - PD0, CAN1_TX - PD1
#else
#error "CANInterface has not been defined for selected board"
#endif

#define CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD   TRUE
#define CAN_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL   TRUE
#define CAN_INTERFACE_THREAD_WORK_AREA_SIZE   1024  // also used in cpp, do not delete

/**
 * CAN interface to receive, distribute and send message
 * @pre CAN pins are configured properly in board.h
 * @usage 1. Create an instance with specific CAN driver
 *        2. Call start() to start the CAN driver and receive thread
 *        3. Set Motor types
 *        4. Invoke set_target_current() to set current of certain motor, the current will be send automatically.
 */
class CANInterface {
public:

    /**
     * Initialize a can interface
     * @param   driver Pointer to CAN driver such as &CAND1
     */
    CANInterface(CANDriver *driver) :
            can_driver(driver), callback_list_count(0)
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
            , errorFeedbackThread(driver, last_error_time)
#endif
            , currentSendThread(driver), processFeedbackThread(driver)
    {}

    /**
     * Start the CAN driver and the receiving thread
     * @param   feedback_prio   Thread priority of the receiving thread
     * @return  A reference to the created thread
     */
    void start(tprio_t feedback_prio, tprio_t send_current_prio);

    static constexpr unsigned MAXIMUM_MOTOR_COUNT = 8;

    enum motor_type_t {
        NONE_MOTOR,
        RM6623,
        M2006,
        GM6020,
        GM6020_HEROH,
        GM3510,
        M3508
    };

    struct motor_feedback_t {

    public:

        motor_type_t type;

        /**
         * Normalized angle
         * @note Viewing from TOP of 6623/2006 motor. 180 <--CCW-- front_angle_raw --CW--> -180
         */
        float actual_angle = 0.0f;     // [degree]

        /**
         * Velocity
         * @note Viewing from TOP of 6623/2006 motor. Positive for CCW. Negative for CW.
         */
        float actual_velocity = 0.0f;  // [degree/s]

        /**
         * Actual current
         * @note Direction is UNKNOWN yet. In reality, it vibrates significantly, and it's not useful for now.
         */
        int actual_current = 0;  // [mA]

        /**
         * Number of round
         * @note Viewing from TOP of 6623/2006 motor. Positive for CCW. Negative for CW.
         */
        int round_count = 0;

        /**
         * Last update time (ms, from system start)
         */
        time_msecs_t last_update_time = 0;

        /**
         * Set current actual angle as the zero reference angle and clear the round count (accumulated angle = 0)
         */
        void reset_front_angle();

        /**
         * Get total angle from the original front angle
         * @return Accumulated angle since last reset_front_angle [degree, positive for CCW viewing from top]
         */
        float accumulated_angle();

        uint16_t last_angle_raw = 0;  // in the range of [0, 8191]

#if CAN_INTERFACE_ENABLE_VELOCITY_DIFFERENTIAL
        // Variables for velocity sampling
        time_msecs_t sample_time_stamp = 0;
        int sample_count = 0;
        int sample_movement_sum = 0;
#endif

    };

    /**
     * Type of callback function
     */
    typedef void (*can_callback_func)(CANRxFrame const *rxmsg);

    /**
     * Set certain motor's type
     * @param id        Target motor's id
     * @param           Motor's type (RM6623, M3508...)
     */
    void set_motor_type(unsigned id, motor_type_t motor_type_);

    /**
     * Register a callback. Message with SID in the given range will be distributed to the given callback function
     * @param sid_lower_bound   Minimal SID (inclusive)
     * @param sid_upper_bound   Maximum SID (inclusive)
     * @param callback_func     The function to callback
     * @return Whether registering succeeded or not
     */
    bool register_callback(uint32_t sid_lower_bound, uint32_t sid_upper_bound, can_callback_func callback_func);

    /**
     * Send a frame
     * @param txmsg   The frame to be sent
     * @return Whether the message has been sent successfully
     */
    bool send_msg(const CANTxFrame *txmsg);

    /**
     * Get target current adress
     * @param id    The motor id.
     */
    int *get_target_current_address(unsigned id);

    /**
     * Set target current for motor
     * @params id               The target motor's id (0-7)
     * @params target_current   Target_current to set
     */
    void set_target_current(unsigned id, int target_current);

    /**
     * Echo certain motor's target_current.
     * @param id        Motor's id
     */
    int echo_target_current(unsigned id);

    /**
     * Event source to broadcast CAN error message
     */
    EVENTSOURCE_DECL(error_event_src);

    /**
     * Get the adress of a certain feedback.
     * @param id  The motor id.
     */
    motor_feedback_t *get_feedback_address(unsigned id);

private:

    CANDriver *can_driver;

    static constexpr unsigned MAXIMUM_REGISTRATION_COUNT = 20;

    static constexpr unsigned int SEND_THREAD_INTERVAL = 1; // can message send interval [ms]

    struct callback_resignation_t {
        uint32_t sid_lower_bound;
        uint32_t sid_upper_bound;
        can_callback_func callback_func;
    } callback_list[MAXIMUM_REGISTRATION_COUNT];

    motor_type_t motor_type_list[MAXIMUM_MOTOR_COUNT] = {NONE_MOTOR};

    unsigned callback_list_count;


#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

public:

    time_msecs_t last_error_time = 0;

private:

    class ErrorFeedbackThread : public chibios_rt::BaseStaticThread<512> {
    public:
        ErrorFeedbackThread(CANDriver *can_driver_, time_msecs_t &last_error_time_) :
                can_driver(can_driver_), last_error_time(last_error_time_) {}
        CANDriver *can_driver;
    private:
        time_msecs_t &last_error_time;

        void main() final;
    };

    ErrorFeedbackThread errorFeedbackThread;

#endif

    class CurrentSendThread : public chibios_rt::BaseStaticThread<512> {
    public:
        explicit CurrentSendThread(CANDriver *can_driver_) :
                can_driver(can_driver_) {}
        CANDriver *can_driver;
        motor_type_t motorType[8];
        int target_current[MAXIMUM_MOTOR_COUNT + 1] = {0};

    private:
        bool send_msg(const CANTxFrame *txmsg);

        void main() final;
    };

    CurrentSendThread currentSendThread;

    class ProcessFeedbackThread : public chibios_rt::BaseStaticThread<CAN_INTERFACE_THREAD_WORK_AREA_SIZE> {
    public:

        explicit ProcessFeedbackThread(CANDriver *can_driver_) :
        can_driver(can_driver_) {}
        CANDriver *can_driver;

        motor_feedback_t feedback[MAXIMUM_MOTOR_COUNT + 1];

    private:
        void main() final;
    };

    ProcessFeedbackThread processFeedbackThread;

    CANConfig can_cfg = {
            CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
            CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
            CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
    };
    static constexpr unsigned TRANSMIT_TIMEOUT_MS = 10;


//    void main() final;
};


#endif //META_INFANTRY_CAN_INTERFACE_H
