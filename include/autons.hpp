#ifndef AUTONS_HPP
#define AUTONS_HPP

#include "lemlib/api.hpp" // IWYU pragma: keep

void pid_setup();

extern lemlib::Chassis chassis;

extern const lemlib::Pose BL_START_POSE;
extern const lemlib::Pose BR_START_POSE;
extern const lemlib::Pose RL_START_POSE;
extern const lemlib::Pose RR_START_POSE;
extern const lemlib::Pose SKILLS_START_POSE;


namespace Autons {

	 void BlueLeft();
	 void BlueRight();
	 void RedLeft();
	 void RedRight();
	 void Skills();
   void TestAuto();

};

#endif
