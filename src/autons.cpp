#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "autons.hpp"

#define GAME_SPEED 110
#define SLOW_SPEED 40
#define SKILLS_SPEED 100


const lemlib::Pose BL_START_POSE(60.5, -58.75, 270);
const lemlib::Pose BR_START_POSE(0, 0, 0);
const lemlib::Pose RL_START_POSE(0, 0, 0);
const lemlib::Pose RR_START_POSE(0, 0, 0);
const lemlib::Pose SKILLS_START_POSE(0, 0, 0);

extern rd::Console console;



///
// START AUTOS 
///

bool isliftup = false;
void Autons::TestAuto() {
    const double targetHeight = isliftup ? 6.5 : 31.0;
    if (lift::move_to_height(targetHeight)) {
        isliftup = !isliftup;
        console.println("lift move started");
    }
}

void Autons::BlueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(BL_START_POSE);


    chassis.moveToPoint(10, 10, 4000, {.maxSpeed=SLOW_SPEED});
    chassis.waitUntilDone();

    chassis.moveToPose(20, 20, 180, 4000, {.maxSpeed=GAME_SPEED});
    chassis.waitUntilDone();

}

void Autons::BlueRight() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(BR_START_POSE);

  chassis.moveToPoint(0, 10, 4000);
  chassis.waitUntilDone();
}

void Autons::RedLeft() {}

void Autons::RedRight() {
  chassis.setPose(RR_START_POSE);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  chassis.moveToPoint(0, 28, 2000);
  lift::up();
  lift::close();
  lift::move_to_height(14);
  chassis.waitUntilDone();

  lift::move_to_height(7, 40);
  lift::wait_until_done();

  lift::open();
  pros::delay(100);
  
  chassis.moveToPose(0, 20, 0, 4000, {.forwards=false, .maxSpeed=SLOW_SPEED});
  chassis.waitUntilDone();

}

void Autons::Skills() {}
