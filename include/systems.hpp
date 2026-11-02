#ifndef SYSTEMS_HPP
#define SYSTEMS_HPP

void initialize_systems(int liftLeftPort, int liftRightPort, int liftRotationPort, char clawOpenPort, char clawDownPort);
bool reset_lift_rotation();
bool move_lift_to_height(double heightInches);
void lift_opcontrol();
void run_claw_test();

#endif
