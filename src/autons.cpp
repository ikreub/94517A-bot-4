#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "dsr.hpp"
#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(13.2, 0.0, 148);    //24,0,266     // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(10.1, 0.0, 72.5);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.9, 0.05, 27.5, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 70.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(7.1, 0.0, 63);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(6.9, 0.0, 52);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 60);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(11_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
  chassis.drive_imu_scaler_set(1.0049);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(48_in, DRIVE_SPEED , true);
  chassis.pid_wait();

  chassis.pid_drive_set(-48_in, DRIVE_SPEED , true);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(3600_deg, 40, ez::raw);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 0);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{12_in, 20_in}, fwd, DRIVE_SPEED},
                        {{24_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{-10_in, 20_in, 90_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{10_in,0_in}, rev, DRIVE_SPEED},{{20_in,30_in}, rev, DRIVE_SPEED}});
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 10_in, 180_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

///
// Calculate offsets for distance sensors
///
void measure_dsr_offsets(){
  DSR::measure_offsets(5);
}

// . . .
// Make your own autonomous functions here!
// . . .

void test(){
  pros::delay(100);
  DSR::reset_tracking(R, B);
  chassis.pid_odom_set({{93.7_in, 46.38_in}, fwd, 40});
  chassis.pid_wait();
  chassis.pid_odom_set({{117.32_in, 22.83_in}, fwd, 40});
  chassis.pid_wait();
  chassis.pid_turn_set(-160_deg, 40);
  chassis.pid_wait();
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{93.7_in, 46.38_in}, fwd, 40});
  chassis.pid_wait();
}

void Right_7(){
  //intake start
  intake.move(127);

  //first three balls
  chassis.pid_odom_set({{5_in, 33_in}, fwd, DRIVE_SPEED}, true);

  //timing for matchloader
  pros::delay(700);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();

  //go back a bit because went too far
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  //align to low goal
  chassis.pid_turn_set({-2_in, 39.5_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();
  MatchLoad.set(false);

  //go to low goal
  chassis.pid_odom_set({{-2_in, 39.5_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait_quick();

  //score in low goal
  LowScore.set(true);
  intake.move(-127);
  outtake.move(-127);
  pros::delay(1500);

  //stop intake
  LowScore.set(false);
  intake.move(0);
  outtake.move(0);

  //go to general long goal/matchload area
  chassis.pid_odom_set({{32_in, 14_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait();

  //align to matchloader
  chassis.pid_turn_set({28_in, -1_in}, fwd, TURN_SPEED);
  MatchLoad.set(true);
  chassis.pid_wait();

  //go to and intake matchloader
  intake.move(127);
  chassis.pid_odom_set({{28_in, -4_in}, fwd, DRIVE_SPEED / 2}, true);
  pros::delay(1300);

  //go to long goal
  chassis.pid_odom_set({{29_in, 28_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick_chain();

  //constant pressure to make sure i am aligned
  chassis.drive_set(-50,-50);
  outtake.move(127);

  //wait until wrong color
  pros::delay(2000);
  outtake.move(0);
  intake.move(0);
  MatchLoad.set(false);

  //descore doesnt really matter
  chassis.pid_odom_set({{22_in, 20_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  //descore actual alignment, please change if necessary
  chassis.pid_odom_set({{{18_in, 26_in}, rev, DRIVE_SPEED}, {{19_in, 44_in}, rev, DRIVE_SPEED}}, true);
  
  //the rest dont matter
  chassis.pid_wait_until_index(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in , DRIVE_SPEED);
  pros::delay(10000);
}

void right_7_rush(){
  //move so that i dont waste time
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(100);

  //start intake
  intake.move(127);

  //reset position
  DSR::reset_tracking(R, B);

  //get three balls
  chassis.pid_odom_set({{95_in, 45_in}, fwd, DRIVE_SPEED});
  //wait until close to trap the balls
  chassis.pid_wait_until_point({95_in, 40_in});
  MatchLoad.set(true);

  //weird bug fix
  chassis.pid_wait_quick();

  //go to general long goal/matchload area
  chassis.pid_odom_set({{123_in, 30_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick();

  //turn to face matchload
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  //reset position
  DSR::reset_tracking(L, F);

  //matchload
  chassis.pid_odom_set({{{121_in, 24_in}, fwd, DRIVE_SPEED}, {{121_in, -4_in}, fwd, DRIVE_SPEED / 2}}, true);
  pros::delay(1400);

  //go to long goal
  chassis.pid_odom_set({{122_in, 50_in}, rev, DRIVE_SPEED}, true);

  //dont score too early
  pros::delay(900);

  //score
  outtake.move(127);
  pros::delay(2100);

  //reset position
  DSR::reset_tracking(L,F);

  //no more score :(
  outtake.move(0);

  //try not to crash
  chassis.pid_odom_set({{114_in, 30_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();

  //descore
  chassis.pid_odom_set({{{112_in, 43_in}, rev, DRIVE_SPEED}, {{113_in, 57_in}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait_until_index(0);
  pros::delay(10000);
}

void skills_102(){

  ////get the six balls in parking barrier
  //reset everything
  chassis.odom_theta_set(-90_deg);
  Wing.set(true);
  Middle.set(false);

  //Push the blocks to the far side
  MatchLoad.set(true);
  pros::delay(100);
  chassis.pid_drive_set(60_in, 30);
  pros::delay(600);

  //put matchloader back up
  chassis.pid_drive_set(0,100);
  MatchLoad.set(false);
  pros::delay(200);

  //start intake
  intake.move(127);

  //get in the parking barrier
  chassis.pid_drive_set(60_in, 90);

  //timing for trapping
  pros::delay(700);
  MatchLoad.set(true);
  pros::delay(200);

  //get the blocks
  chassis.pid_drive_set(42_in, 80, true);
  chassis.pid_wait_quick_chain();

  //turn to get position
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SWING_SPEED, 0);
  chassis.pid_wait_quick();
  DSR::reset_tracking(R, F);

  ////middle goal scoring
  //move to the right area
  chassis.pid_odom_set({{{48_in, 24_in}, rev, DRIVE_SPEED},
                                      {{62_in, 36_in}, rev, DRIVE_SPEED},
                                      {{58_in, 51_in}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait_quick_chain();

  //fully align
  MatchLoad.set(false);
  chassis.drive_set(-60,-60);
  pros::delay(700);
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait();

  //constant pressure for scoring
  chassis.drive_set(-20,-20);
  Middle.set(true);

  //finicky stuff to make sure it doesn't jam
  intake.move(-60);
  outtake.move(-90);
  pros::delay(200);
  intake.move(90);
  pros::delay(2000);

  //get the last ball
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-20,-20);
  pros::delay(200);
  Middle.set(false);
  outtake.move(0);
  chassis.pid_drive_set(9_in, DRIVE_SPEED / 2);
  chassis.pid_wait_quick();

  //realign
  chassis.pid_drive_set(-13_in,40);
  pros::delay(600);
  Middle.set(true);
  outtake.move(-90);
  pros::delay(1300);
  intake.move(127);
  Middle.set(false);
  outtake.move(0);

  //// Matchloading and long goal scoring
  //go to general area
  chassis.pid_odom_set({{23_in, 23_in}, fwd, DRIVE_SPEED}, true);

  //get three extra balls
  pros::delay(500);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();

  //turn to face matchloader
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  //reset position
  DSR::reset_tracking(R, F);

  //go to long goal to score three balls
  chassis.pid_odom_set({{19_in, 36_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-40,-40);
  outtake.move(127);
  pros::delay(1500);
  outtake.move(0);

  //matchload
  chassis.pid_odom_set({{{19_in, 20_in}, fwd, DRIVE_SPEED},{{19_in, 0_in}, fwd, DRIVE_SPEED / 2}});
  pros::delay(2400);

  //go to other side
  chassis.pid_odom_set({{{4_in, 35_in}, rev, DRIVE_SPEED},{{5_in, 95_in}, rev, DRIVE_SPEED}, {{20_in, 110_in}, rev, DRIVE_SPEED}});
  chassis.pid_wait_quick_chain();

  //turn to face matchloader
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  //reset position
  DSR::reset_tracking(L, F);

  //score in long goal
  chassis.pid_odom_set({{19_in, 85_in}, rev, DRIVE_SPEED / 2}, true);

  //dont score too early
  pros::delay(750);

  //score
  outtake.move(127);
  pros::delay(2000);
  outtake.move(0);

  //reset position
  DSR::reset_tracking(L, F);

  // matchload
  chassis.pid_odom_set({{{19_in, 120_in}, fwd, DRIVE_SPEED}, {{19_in, 144_in}, fwd, DRIVE_SPEED / 2}}, true);
  pros::delay(2400);

  //go to long goal
  chassis.pid_odom_set({{19_in, 85_in}, rev, DRIVE_SPEED / 2}, true);

  //dont score too early
  pros::delay(1000);

  //score
  outtake.move(127);
  pros::delay(1000);

  //change speed to score more
  outtake.move(90);
  pros::delay(1000);
  outtake.move(0);

  //reset position
  DSR::reset_tracking(L, F);

  //turn to get extra balls and reset matchloader
  chassis.pid_turn_set(105_deg, TURN_SPEED);
  MatchLoad.set(false);
  chassis.pid_wait();

  //get the three balls
  chassis.pid_odom_set({{43_in, 100_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait_quick_chain();

  //get extra balls with the power of pneumatics
  MatchLoad.set(true);

  //go to general area
  chassis.pid_odom_set({{{82_in, 120_in}, fwd, DRIVE_SPEED}, {{112_in, 120_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_quick();

  //turn to face match loader
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  //reset position
  DSR::reset_tracking(R, F);

  //go to long goal to score extra balls
  chassis.pid_odom_set({{122_in, 100_in}, rev, DRIVE_SPEED / 2});

  //dont score too early
  pros::delay(800);

  //score
  outtake.move(127);
  pros::delay(1200);
  outtake.move(0);

  //reset position
  DSR::reset_tracking(R, F);

  //Matchload
  chassis.pid_odom_set({{{121_in, 120_in}, fwd, DRIVE_SPEED}, {{121_in, 144_in}, fwd, DRIVE_SPEED / 2}}, true);
  pros::delay(2400);
  
  //go around the long goal to go to the other side
  chassis.pid_odom_set({{{139_in, 100_in}, rev, DRIVE_SPEED}, {{139_in, 40_in}, rev, DRIVE_SPEED}, {{125_in, 34_in}, rev, DRIVE_SPEED}}, true);
  chassis.pid_wait_quick();

  //turn to face the match loader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  //reset position
  DSR::reset_tracking(L, F);

  //go to long goal
  chassis.pid_odom_set({{121_in, 45_in}, rev, DRIVE_SPEED / 2});

  //dont score too early
  pros::delay(800);

  //score
  outtake.move(127);
  pros::delay(2000);
  outtake.move(0);

  //reset position
  DSR::reset_tracking(L, F);

  //matchload
  chassis.pid_odom_set({{{121_in, 24_in}, fwd, DRIVE_SPEED}, {{121_in, 0_in}, fwd, DRIVE_SPEED / 2}}, true);
  pros::delay(2400);

  //go to long goal
  chassis.pid_odom_set({{122_in, 45_in} ,rev, DRIVE_SPEED / 2});

  //dont score too early
  pros::delay(900);

  //reset match loader
  MatchLoad.set(false);

  //score
  outtake.move(127);
  pros::delay(1000);

  //change speed to score more
  outtake.move(90);
  pros::delay(1000);
  outtake.move(0);

  //go to parking barrier
  chassis.pid_odom_set({{{122_in, 20_in}, fwd, DRIVE_SPEED}, {{108_in, 8_in}, fwd, DRIVE_SPEED}, {{98_in, 6_in}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait_quick_chain();
  Wing.set(true);

  // turn to optimal angle
  chassis.pid_turn_set(-100_deg, TURN_SPEED);
  chassis.pid_wait();

  //get in the barrier
  chassis.pid_drive_set(30_in, 127, true, false);
  chassis.pid_wait();
}

void Left_7(){
  //intake start
  intake.move(127);

  //first three balls
  chassis.pid_odom_set({{-5_in, 33_in}, fwd, DRIVE_SPEED}, true);

  //timing for matchloader
  pros::delay(700);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();

  //go back a bit because went too far
  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  //align to middle goal
  chassis.pid_turn_set({4_in, 48.5_in}, rev, DRIVE_SPEED);
  chassis.pid_wait();
  MatchLoad.set(false);

  //go to middle goal
  chassis.pid_odom_set({{6_in, 49.5_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick();

  //score in middle goal
  Middle.set(true);
  outtake.move(-127);
  intake.move(-60);
  pros::delay(300);
  intake.move(127);
  pros::delay(1500);

  //stop intake
  Middle.set(false);
  intake.move(0);
  outtake.move(0);

  //go to general long goal/matchload area
  chassis.pid_odom_set({{-32_in, 14_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  //align to matchloader
  chassis.pid_turn_set({-33_in, -1_in}, fwd, TURN_SPEED);
  MatchLoad.set(true);
  chassis.pid_wait();

  //go to and intake matchloader
  intake.move(127);
  chassis.pid_odom_set({{-33_in, -4_in}, fwd, DRIVE_SPEED / 2}, true);
  pros::delay(1300);

  //go to long goal
  chassis.pid_odom_set({{-33_in, 28_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick_chain();

  //constant pressure to make sure i am aligned
  chassis.drive_set(-50,-50);
  outtake.move(127);

  //wait until wrong color
  pros::delay(2000);
  outtake.move(0);
  intake.move(0);
  MatchLoad.set(false);

  //descore doesnt really matter
  chassis.pid_odom_set({{-22_in, 16_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  //descore actual alignment, please change if necessary
  chassis.pid_odom_set({{{-20_in, 26_in}, fwd, DRIVE_SPEED}, {{-21_in, 44_in}, fwd, DRIVE_SPEED}}, true);
  
  //the rest dont matter
  chassis.pid_wait_until_index(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(10_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in , DRIVE_SPEED);
  pros::delay(10000);
}

void left_7_rush(){

  //move forward a little to not waste time
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(100);

  //start intake
  intake.move(127);

  //reset tracking
  DSR::reset_tracking(L, B);

  //go to the three balls
  chassis.pid_odom_set({{45_in, 45_in}, fwd, DRIVE_SPEED});

  //wait until close and trap the balls
  chassis.pid_wait_until_point({45,40});
  MatchLoad.set(true);
  pros::delay(300);

  //go to general area
  chassis.pid_odom_set({{20_in, 30_in}, rev, DRIVE_SPEED}, true);
  chassis.pid_wait_quick_chain();

  //turn to matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  MatchLoad.set(true);

  //reset position
  DSR::reset_tracking(R, F);

  //match load
  chassis.pid_odom_set({{{20_in, 24_in}, fwd, DRIVE_SPEED}, {{20_in, -4_in}, fwd, DRIVE_SPEED / 2}}, true);
  pros::delay(1400);

  // go to long goal
  chassis.pid_odom_set({{20_in, 50_in}, rev, DRIVE_SPEED}, true);

  //dont score too early
  pros::delay(900);

  //score
  intake.move(127);
  outtake.move(127);
  pros::delay(2100);

  //reset match loader so that i dont cross
  MatchLoad.set(false);
  outtake.move(0);

  //try not to crash
  chassis.pid_odom_set({{25_in, 30_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();

  //go to descore
  chassis.pid_odom_set({{{31_in, 43_in}, fwd, DRIVE_SPEED}, {{29_in, 55_in}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait_quick_chain();

  //turn for better descore
  chassis.pid_turn_set(10_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // hold position
  chassis.pid_drive_set(0_in, 110);
  pros::delay(10000);
}

void move_a_bit(){
  chassis.drive_set(60,60);
  pros::delay(500);
  chassis.drive_set(0,0);
}

void dont_do_anything(){
  pros::delay(14000);
}

void SAWP(){
  chassis.odom_theta_set(-90_deg);
  intake.move(127);
  chassis.pid_drive_set(7_in, DRIVE_SPEED);
  pros::delay(500);
  DSR::reset_tracking(B, L);
  chassis.pid_odom_set({{118_in, 27_in}, rev, 127}, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  MatchLoad.set(true);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{121_in, 20_in}, fwd, DRIVE_SPEED},
  {{121_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1200);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{121_in, 50_in}, rev, DRIVE_SPEED}, true);
  pros::delay(900);
  outtake.move(127);
  MatchLoad.set(false);
  pros::delay(1301);
  outtake.move(0);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{100_in, 43_in}, fwd, 120}, 
                        {{56_in, 43_in}, fwd, 120}, 
                        {{32_in, 30_in}, fwd, 120}}, true);
  chassis.pid_wait_until_index_started(1);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{18_in, 50_in}, rev, 120}, true);
  pros::delay(300);
  outtake.move(127);
  pros::delay(1100);
  outtake.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{19_in, 24_in}, fwd, 120},
                        {{19_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1400);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{20_in, 20_in}, rev, 120},
                        {{54_in, 52_in}, rev, 120}}, true);
  chassis.pid_wait_quick_chain();
  Middle.set(true);
  chassis.drive_set(-40,-40);
  intake.move(0);
  outtake.move(-127);
  intake.move(-80);
  pros::delay(300);
  intake.move(127);
  pros::delay(3000);
}

void SAWP2(){
  chassis.odom_theta_set(90_deg);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  pros::delay(100);
  DSR::reset_tracking(F, R);
  chassis.pid_odom_set({{118_in, 24_in}, fwd, 127}, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  MatchLoad.set(true);
  intake.move(127);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{121_in, 20_in}, fwd, DRIVE_SPEED},
  {{121_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1200);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{121_in, 50_in}, rev, DRIVE_SPEED}, true);
  pros::delay(900);
  outtake.move(127);
  MatchLoad.set(false);
  pros::delay(1300);
  outtake.move(0);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{100_in, 43_in}, fwd, DRIVE_SPEED}, 
                        {{56_in, 43_in}, fwd, 120}, 
                        {{32_in, 30_in}, fwd, DRIVE_SPEED}}, true);
  chassis.pid_wait_until_index_started(1);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{18_in, 50_in}, rev, DRIVE_SPEED}, true);
  pros::delay(600);
  outtake.move(127);
  pros::delay(1100);
  outtake.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{19_in, 24_in}, fwd, 120},
                        {{19_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1400);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{20_in, 20_in}, rev, 120},
                        {{53_in, 51_in}, rev, 120}}, true);
  chassis.pid_wait_quick_chain();
  Middle.set(true);
  chassis.drive_set(-50,-50);
  outtake.move(-127);
  intake.move(-80);
  pros::delay(200);
  intake.move(127);
  pros::delay(1000);
}

void Right_counter_7(){
  chassis.odom_theta_set(90_deg);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  pros::delay(100);
  DSR::reset_tracking(F, R);
  chassis.pid_odom_set({{118_in, 24_in}, fwd, 127}, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  MatchLoad.set(true);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{121_in, 20_in}, fwd, DRIVE_SPEED},
  {{121_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1200);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{121_in, 50_in}, rev, DRIVE_SPEED}, true);
  pros::delay(700);
  outtake.move(127);
  MatchLoad.set(false);
  pros::delay(1000);
  outtake.move(0);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{101_in, 43_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  MatchLoad.set(true);
  pros::delay(500);
  MatchLoad.set(false);
  LowScore.set(true);
  chassis.pid_odom_set({{93.5_in, 49.5_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait_quick();
  intake.move(-127);
  outtake.move(-127);
  pros::delay(1000);
  chassis.pid_odom_set({{144_in, 47_in}, rev, DRIVE_SPEED});
  pros::delay(750);
  Wing.set(true);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  Wing.set(false);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  pros::delay(10000);
}

void Left_counter_7(){
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(100);
  intake.move(127);
  DSR::reset_tracking(L, B);
  chassis.pid_odom_set({{45_in, 45_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  MatchLoad.set(true);
  chassis.pid_odom_set({{{20_in,24_in}, rev, DRIVE_SPEED},
                        {{18_in, 40_in}, rev, DRIVE_SPEED}});
  chassis.pid_wait_quick_chain();
  outtake.move(127);
  chassis.drive_set(-20,-20);
  pros::delay(1000);
  outtake.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{19_in, 24_in}, fwd, 120},
                        {{19_in, -4_in}, fwd, DRIVE_SPEED}}, true);
  pros::delay(1400);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{20_in, 20_in}, rev, 120},
                        {{53_in, 51_in}, rev, 120}}, true);
  chassis.pid_wait_quick_chain();
  Middle.set(true);
  chassis.drive_set(-50,-50);
  outtake.move(-127);
  pros::delay(1500);
  Middle.set(false);
  MatchLoad.set(false);
  Wing.set(true);
  chassis.pid_odom_set({{20_in, 47_in}, fwd, DRIVE_SPEED});
  pros::delay(800);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  Wing.set(false);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(10_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(0);
  chassis.pid_drive_set(0, 110);
  pros::delay(10000);
}