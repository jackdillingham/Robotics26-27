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
const lemlib::Pose SKILLS_START_POSE(-57.4000, 58.5000, 270.000);

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

ASSET(pinnacle_01_txt);
ASSET(pinnacle_02_txt);
ASSET(pinnacle_03_txt);
ASSET(pinnacle_04_txt);
ASSET(pinnacle_05_txt);

void Autons::Skills() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  
  start_mcl_auton(SKILLS_START_POSE);
  
//chassis.setPose(-57.4000, 58.5000, 270.000);
// Movement 1: Score prelaod
chassis.turnToHeading(270.000, 2000, {}, false);
chassis.follow(pinnacle_01_txt, 10, 6022, false, true);
chassis.waitUntil(1.5109); // Lift to 10in
lift::move_to_height(10, 127);
chassis.waitUntil(28.7080); // Score preload and return lift
lift::move_to_height(6.5, 40);
pros::delay(400);
lift::open();
pros::delay(200);
chassis.waitUntilDone();
chassis.turnToHeading(0.000, 2000, {}, false);

// Movement 2: Turn around
chassis.turnToHeading(0.000, 2000, {}, false);
chassis.follow(pinnacle_02_txt, 10, 6107, true, true);
chassis.waitUntilDone();
chassis.turnToHeading(90.000, 2000, {}, false);

// Movement 3: Matchload
chassis.turnToHeading(90.000, 2000, {}, false);
chassis.follow(pinnacle_03_txt, 10, 5700, false, true);
chassis.waitUntil(0.0000); // Grab matchload
lift::close();
chassis.waitUntilDone();

// Movement 4: Move towards center and turn around
chassis.turnToHeading(90.000, 2000, {}, false);
chassis.follow(pinnacle_04_txt, 10, 13409, true, true);
chassis.waitUntilDone();

// Movement 5: Score matchload on center
chassis.turnToHeading(270.000, 2000, {}, false);
chassis.follow(pinnacle_05_txt, 10, 5350, false, true);
chassis.waitUntil(0.0000); // Lift up 4 bar
lift::move_to_height(18, 127);
chassis.waitUntil(23.5000); // Intake on
lift::move_to_height(15, 40);
pros::delay(500);
lift::open();
chassis.waitUntilDone();
}
