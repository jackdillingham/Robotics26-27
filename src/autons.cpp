#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "autons.hpp"

const lemlib::Pose BL_START_POSE(0, 0, 0);

void pid_setup() {
    
}

void Autons::BlueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(BL_START_POSE);


    chassis.moveToPoint(10, 10, 4000, {.maxSpeed=40});
    chassis.waitUntilDone();
}

void Autons::BlueRight() {}

void Autons::RedLeft() {}

void Autons::RedRight() {}

void Autons::Skills() {}