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




///
// START AUTOS 
///

void Autons::BlueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(BL_START_POSE);


    chassis.moveToPoint(10, 10, 4000, {.maxSpeed=SLOW_SPEED});
    chassis.waitUntilDone();

    chassis.moveToPose(20, 20, 180, 4000, {.maxSpeed=GAME_SPEED});
    chassis.waitUntilDone();

}

void Autons::BlueRight() {}

void Autons::RedLeft() {}

void Autons::RedRight() {}

void Autons::Skills() {}
