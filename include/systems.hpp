#ifndef SYSTEMS_HPP
#define SYSTEMS_HPP

namespace lift {

void initialize_systems(int liftLeftPort, int liftRightPort, int liftRotationPort, char clawOpenPort, char clawDownPort);
bool reset_rotation();
bool move_to_height(double heightInches);
bool move_to_height(double heightInches, int maxSpeed);
void wait_until_done();
void opcontrol();
void open();
void close();
void up();
void down();
void toggle_claw_up_down();
void toggle_claw_open_closed();
void hold();
void coast();
void run_claw_test();

}

#endif
