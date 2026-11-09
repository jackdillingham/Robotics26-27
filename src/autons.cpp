#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "autons.hpp"

#define GAME_SPEED 110
#define SLOW_SPEED 40
#define SKILLS_SPEED 100


const lemlib::Pose BL_START_POSE(60.5, -58.75, 270);
const lemlib::Pose BR_START_POSE(0, 0, 0);
const lemlib::Pose RL_START_POSE(49.75, -61.5, 90);
const lemlib::Pose RR_START_POSE(49.75, 61.5, 90);
const lemlib::Pose SKILLS_START_POSE(0, 0, 0);

extern rd::Console console;



///
// START AUTOS 
///

bool isliftup = false;
void Autons::TestAuto() {
    mcl.disable(); // Lift-only test has no known field starting pose.
    lift::move_to_height(10);
    lift::wait_until_done();
}

void Autons::BlueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    start_mcl_auton(BL_START_POSE);


    chassis.moveToPoint(10, 10, 4000, {.maxSpeed=SLOW_SPEED});
    chassis.waitUntilDone();

    chassis.moveToPose(20, 20, 180, 4000, {.maxSpeed=GAME_SPEED});
    chassis.waitUntilDone();
    mcl.disable();
}

void Autons::BlueRight() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  start_mcl_auton(BR_START_POSE);

  chassis.moveToPoint(0, 10, 4000);
  chassis.waitUntilDone();
  mcl.disable();
}

void Autons::RedLeft() {
  start_mcl_auton(RL_START_POSE);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  lift::move_to_height(10, 127);

  chassis.moveToPose(27, -40, 0, 2500, {.forwards=false});
  //lift::up();
  //lift::close();
  pros::delay(100);
  chassis.waitUntilDone();

  lift::move_to_height(7, 40);
  pros::delay(100);

  lift::open();
  //pros::delay(100);
  //
  lift::move_to_height(6.5, 127);
  pros::delay(700);
  mcl.disable();
  
  //chassis.moveToPose(27, -64, 90, 4000, {.maxSpeed=SLOW_SPEED});
  //chassis.waitUntilDone();


}

void Autons::RedRight() {
  start_mcl_auton(RR_START_POSE);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  lift::move_to_height(10, 127);

  chassis.moveToPose(27, 40, 0, 2500, {.forwards=false});
  //lift::up();
  //lift::close();
  pros::delay(100);
  chassis.waitUntilDone();

  lift::move_to_height(7, 40);
  pros::delay(100);

  lift::open();
  //pros::delay(100);
  //
  lift::move_to_height(6.5, 127);
  pros::delay(700);
  mcl.disable();
  
  //chassis.moveToPose(27, -64, 90, 4000, {.maxSpeed=SLOW_SPEED});
  //chassis.waitUntilDone();



}

void Autons::Skills() { mcl.disable(); }
