#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <algorithm> // IWYU pragma: keep

///
// PORTS
///

// drive
#define BRD_PORT -2
#define MRD_PORT 3
#define FRD_PORT 4
#define BLD_PORT -5
#define MLD_PORT -6
#define FLD_PORT 7

// lift
#define LIFTL_PORT -16
#define LIFTR_PORT 1
#define LIFT_ROTATION_PORT 15

// odom
#define H_ODOM_PORT 19
#define V_ODOM_PORT 18

// intake
#define INTAKE_PORT 17  // NOT CODED

// pnuematics
#define CLAW_OPEN_PORT 'B'
#define CLAW_DOWN_PORT 'A'




// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// left motor group
pros::MotorGroup left_motor_group({BLD_PORT, MLD_PORT, FLD_PORT}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({BRD_PORT, MRD_PORT, FRD_PORT}, pros::MotorGears::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);


// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(H_ODOM_PORT);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(V_ODOM_PORT);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Distance distFront(12);
pros::Distance distBack(13);
pros::Distance distLeft(14);
pros::Distance distRight(20);

constexpr bool MCL_METAL_PERIMETER = false;
constexpr double MCL_FIELD_SIDE = MCL_METAL_PERIMETER ? 140.50 : 140.40;
// PLACEHOLDER OFFSETS BELOW: measure lens forward/right from the odometry center
// and verify all four facing angles before changing this flag to true.
constexpr bool MCL_GEOMETRY_CONFIGURED = true;
localization::MCL mcl(chassis, {
    {&distFront, -4, 6.25,   0}, // PLACEHOLDER forward/right; front-facing
    {&distBack,  4.125, -7, 180}, // PLACEHOLDER forward/right; rear-facing
    {&distLeft,  -6.5, -2.25, 270}, // PLACEHOLDER forward/right; left-facing
    {&distRight, 6.5, -2.25,  90}  // PLACEHOLDER forward/right; right-facing
}, {MCL_FIELD_SIDE, MCL_FIELD_SIDE, 0, 0}); // Origin at field center.

// Used by both competition autonomous and RoboDash's direct routine calls.
void start_mcl_auton(lemlib::Pose startPose) {
    mcl.disable();
    mcl.setPose(startPose); // Reset LemLib and the particle baseline together.
    if (MCL_GEOMETRY_CONFIGURED) mcl.enable();
}

rd::Selector selector({
    {"Testing Auto", Autons::TestAuto, "", 60},
    {"Red Right", Autons::RedRight, "", 0},
    {"Red Left", Autons::RedLeft, "", 0},
    //{"Blue Right", Autons::BlueRight, "", 220},
    //{"Blue Left", Autons::BlueLeft, "", 220},
    {"Skills", Autons::Skills, "", 120}
});

rd::Console console;

void run_selected_auto(lv_event_t *) {
    //Autons::TestAuto();
    selector.run_auton();
    console.println("auto ran");
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
bool centerPressed = false;
void on_center_button() {
	//centerPressed = !centerPressed;
	if (centerPressed) {
		console.println("running autonomous");
		Autons::Skills();
    centerPressed = true;
    
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");
  //
  chassis.calibrate();
  mcl.start(); // Background task starts once; corrections enabled per auton.
  if (!MCL_GEOMETRY_CONFIGURED) console.println("MCL disabled: sensor offsets are placeholders");
  lift::initialize_systems(LIFTL_PORT, LIFTR_PORT, LIFT_ROTATION_PORT, CLAW_OPEN_PORT, CLAW_DOWN_PORT);
  lift::close();
  lift::up();
  intake::initialize(INTAKE_PORT);
  if (!lift::reset_rotation()) {
    console.println("lift rotation reset failed");
  }

	pros::lcd::register_btn1_cb(on_center_button);

    rd_view_t *run_auto_view = rd_view_create("Run Auto Button");
    lv_obj_t *run_auto_button = lv_btn_create(rd_view_obj(run_auto_view));
    lv_obj_add_event_cb(run_auto_button, run_selected_auto, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *run_auto_label = lv_label_create(run_auto_button);
    lv_label_set_text(run_auto_label, "Run Selected Auto");
    lv_obj_center(run_auto_label);
    lv_obj_center(run_auto_button);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() { mcl.disable(); }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  
  mcl.enable();

  lemlib::Pose startPoseTest(23, 23, 0);
  mcl.setPose(startPoseTest);

  while (true) {
    console.clear();

    lemlib::Pose currentPose = chassis.getPose();
    console.printf("The robot's position is x:%f, y:%f, theta:%f\n", currentPose.x, currentPose.y, currentPose.theta);
    pros::delay(10);
  }

  //selector.run_auton();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  //mcl.disable();
  lemlib::Pose startPoseTest(0,0,0);
  start_mcl_auton(startPoseTest);

  lift::up();
  lift::close();

  //console.println("driver ctrl started");

  //pros::delay(1000);
  //Autons::Skills();

  while (false) {
    const auto pose = chassis.getPose();
    char line[80];
    std::snprintf(line, sizeof(line), "X: %.2f  Y: %.2f  H: %.1f",
                  pose.x, pose.y, pose.theta);
    console.clear();
    console.println(line);
    pros::delay(200);
  }

  //return;

	while (true) {
        lift::opcontrol();
        intake::opcontrol();
        
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25); // Run for 20 ms then update
	}
}
