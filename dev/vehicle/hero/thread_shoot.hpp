//
// Created by 钱晨 on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_SHOOT_H
#define META_INFANTRY_THREAD_SHOOT_H

#include <math.h>

/**
 * @name ShootThread
 * @brief Thread to control shooter
 * @pre Remote interpreter starts receiving
 * @pre GimbalInterface(Shoot) get properly init()
 * @pre GimbalThread starts
 */


/**
 * @brief The Simple graph of the loaded bullet[4]
 *
 *          addition[3]        _
 *          __________        | |
 *          \         \       | |
 *           \         \    __| |__
 *            \ last[2] \  /       \
 *             \  ________/         \_______
 *              \ |_______           _______|--------------|
 *               \ second \  ____   /                      |
 *                \  [1]  / /[0] \ \     Launch!           |
 *                 \     / /first \ \     HERO XIUXIUXIU!  |
 *                  \___/_/________\_\_____________________|
 *                            \        /
 *                            |_______|
 *
 */

 static bool loader_stop[3] = {TRUE, TRUE, TRUE};
 static bool plate_stop[3] = {TRUE, TRUE, TRUE};
//static float Loader_handle_stuck_target_angle;

 class ShootThread : public chibios_rt::BaseStaticThread<1024>{

     static constexpr unsigned int SHOOT_THREAD_INTERVAL = 5; // PID calculation interval [ms]

     static constexpr  float  COMMON_SHOOT_SPEED = 5;

     bool pc_right_pressed = false; // local variable to control one click of right button of mouse

     float plate_target_angle = 10.0f;
     float bullet_target_angle = 0.0f;


     bool loaded_bullet[4] = {FALSE,FALSE,FALSE,FALSE};

     void main() final {

         setName("shoot");

         Shoot::feedback[3].actual_angle = 10.0f; // Initialize the actual angle. Then plate won't move when start.

         Shoot::change_pid_params(GIMBAL_PID_BULLET_LOADER_A2V_PARAMS, GIMBAL_PID_BULLET_LOADER_V2I_PARAMS, GIMBAL_PID_BULLET_PLATE_A2V_PARAMS, GIMBAL_PID_BULLET_PLATE_V2I_PARAMS);

         while (!shouldTerminate()) {

             loaded_bullet[2] = (bool) palReadPad(GPIOE,GPIOE_PIN4); // check if the last bullet place is full or not.

             // get the correct angle
             if (plate_target_angle > 180.0f && Shoot::feedback[3].actual_angle < plate_target_angle -360.0f) plate_target_angle -= 360.0f;
             if (plate_target_angle < -180.0f && Shoot::feedback[3].actual_angle > plate_target_angle + 360.0f - 3.0f) plate_target_angle += 360.0f;
             if (bullet_target_angle > 180.0f && Shoot::feedback[2].actual_angle < bullet_target_angle -360.0f) bullet_target_angle -= 360.0f;
             if (bullet_target_angle < -180.0f && Shoot::feedback[2].actual_angle > bullet_target_angle + 360.0f - 3.0f) bullet_target_angle += 360.0f;
             /*** Log the motor status, whether it is moving or not.***/

             // check if the loader is moving. Three variables sequence increase the latter system's stability.
             loader_stop[0] = loader_stop [1];
             loader_stop[1] = loader_stop [2];
             loader_stop[2] = fabs(bullet_target_angle - Shoot::feedback[2].actual_angle) < 5.0f || fabs(bullet_target_angle + 360.0f - Shoot::feedback[2].actual_angle) < 5.0f || fabs(bullet_target_angle - 360.0f - Shoot::feedback[2].actual_angle) < 5.0f;

             // Maybe checking the angle is more reliable than checking the velocity.
             // Because when start the velocity could be small too.

             //same as above
             plate_stop[0] = plate_stop [1];
             plate_stop[1] = plate_stop [2];
             plate_stop[2] = (plate_target_angle - Shoot::feedback[3].actual_angle)< 2.0f;

             /*** The bullet plate logic ***/
             // If the loader is stopped and the last place of bullet loader is not full.
             // When first bullet place is empty, it will also load a ball in prepare of the 2-step shoot.
             if (plate_stop[0] == TRUE && plate_stop[1] == TRUE && plate_stop[2] == TRUE && (loaded_bullet[2] == FALSE || (loaded_bullet[0] == FALSE && loaded_bullet[3] == FALSE))) {
                 plate_target_angle += 36.0f;
             }
             Shoot::calc_plate(Shoot::feedback[3].actual_velocity, plate_target_angle);

             /*** This part is for loader automatically fill (prevent void shot) ***/
             // If the loader is stopped and the first place of bullet loader is not full, automatically load.
             if (loader_stop[0] == TRUE && loader_stop[1] == TRUE && loader_stop[2] == TRUE && loaded_bullet[0]==FALSE && loaded_bullet[2] == TRUE) {
                 bullet_target_angle += 72.0f;
                 loaded_bullet[0] = loaded_bullet[1];
                 loaded_bullet[1] = loaded_bullet[2];
                 loaded_bullet[2] = loaded_bullet[3]; // though loaded_2 will automatically refreshed, but it won't update in this cycle.
                 loaded_bullet[3] = FALSE;
             }

             // Happened when first place is empty and firing. (Turn doubled angle)
             // If the last place of loader is full there are still ball passed by, stands for another ball is coming
             // GPIOE_PIN5 stands for the upper sensor. (Near the plate)
             if (loaded_bullet[2] == TRUE && (bool)palReadPad(GPIOE,GPIOE_PIN5)){
                 loaded_bullet[3] = TRUE;
             }

             /*** Shoot Logic ***/
             if (!StateHandler::remoteDisconnected() && !StateHandler::gimbalSeriousErrorOccured()){
                 if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP){

                     // Friction wheels
                     Shoot::set_friction_wheels(0);

                     // Bullet loader motor
                     if (Remote::rc.ch1 > 0.5) {
                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(loader_stop[0] && loader_stop[1] && loader_stop[2]) {
                             if (loaded_bullet[0] == FALSE){ // here we may need more methods to check.
                                 // Though the algorithm designed this,
                                 // it could be more safer to have a new sensor.
                                 bullet_target_angle += 144.0f;
                                 loaded_bullet[0] = loaded_bullet[2];
                                 loaded_bullet[1] = loaded_bullet[3];
                                 loaded_bullet[3] = FALSE; // loaded bullet 2 would be refreshed in real time.
                             } else {
                                 bullet_target_angle += 72.0f;
                                 loaded_bullet[0] = loaded_bullet [1];
                                 loaded_bullet[1] = loaded_bullet [2];
                                 loaded_bullet[2] = loaded_bullet [3];
                                 loaded_bullet[3] = FALSE; // Generalize the update process
                             }
                         }
                     }

                 } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                     // Friction wheels
                     Shoot::set_friction_wheels(GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE);

                     // Bullet loader motor
                     if (Remote::rc.ch3 < -0.1) {
                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(loader_stop[0] && loader_stop[1] && loader_stop[2]) {
                             if (loaded_bullet[0] == FALSE){ // when the first place is empty
                                 bullet_target_angle += 144.0f;
                                 loaded_bullet[0] = loaded_bullet[2];
                                 loaded_bullet[1] = loaded_bullet[3];
                                 loaded_bullet[3] = FALSE; // loaded bullet 2 would be refreshed in real time.
                             } else {
                                 bullet_target_angle += 72.0f;
                                 loaded_bullet[0] = loaded_bullet [1];
                                 loaded_bullet[1] = loaded_bullet [2];
                                 loaded_bullet[2] = loaded_bullet [3];
                                 loaded_bullet[3] = FALSE; // Generalize the update process
                             }
                         }

                     }

                 } else if (Remote::rc.s1 == Remote::S_DOWN) { // PC control mode

                     // Bullet loader motor
                     if (Remote::mouse.press_left) {

                         if (Shoot::fw_duty_cycle == 0) {
                             Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             sleep(TIME_I2MS(500));
                         }
                         if(loader_stop[0] && loader_stop[1] && loader_stop[2]) {
                             if (loaded_bullet[0] == FALSE){
                                 bullet_target_angle += 144.0f;
                                 loaded_bullet[0] = loaded_bullet[2];
                                 loaded_bullet[1] = loaded_bullet[3];
                                 loaded_bullet[3] = FALSE; // loaded bullet 2 would be refreshed in real time.
                             } else {
                                 bullet_target_angle += 72.0f;
                                 loaded_bullet[0] = loaded_bullet [1];
                                 loaded_bullet[1] = loaded_bullet [2];
                                 loaded_bullet[2] = loaded_bullet [3];
                                 loaded_bullet[3] = FALSE; // Generalize the update process
                             }
                         }
                     }

                     // Friction wheels
                     if (Remote::mouse.press_right) {
                         if (!pc_right_pressed) {
                             if (Shoot::fw_duty_cycle == 0) {
                                 Shoot::set_friction_wheels(GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE);
                             } else {
                                 Shoot::set_friction_wheels(0);
                             }
                             pc_right_pressed = true;
                         }
                     } else {
                         if (pc_right_pressed) {
                             pc_right_pressed = false;
                         }
                     }
                 } else {

                     Shoot::set_friction_wheels(0);
                     Shoot::target_current[Shoot::BULLET] = 0;
                     Shoot::target_current[Shoot::PLATE] = 0;

                 }

             } else {
                 Shoot::set_friction_wheels(0);
                 Shoot::target_current[Shoot::BULLET] = 0;
                 Shoot::target_current[Shoot::PLATE] = 0;
             }

             /*** before calculate it, firstly check if the loader is stucked. ***/
             // If stucked, the target angle would change.
             // If not, the target angle would stayed the same.
             if(StateHandler::bulletLoaderStuck()){
                 int count = 0;
                 while(count < 20)
                 {
                     Shoot::target_current[Shoot::BULLET] = -1000; // Just let it role back. So no need to use PID and reach a fixed angle.
                     Shoot::target_current[Shoot::PLATE] = 0;
                     count++;
                     sleep(TIME_MS2I(SHOOT_THREAD_INTERVAL));
                 }
                 StateHandler::bulletLoaderSmooth();
             }
             Shoot::calc_bullet(Shoot::feedback[2].actual_velocity, bullet_target_angle);

             sleep(TIME_MS2I(SHOOT_THREAD_INTERVAL));
         }
     }
 };
#endif //META_INFANTRY_THREAD_SHOOT_HPP

