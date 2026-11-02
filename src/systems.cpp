#include "main.h" // IWYU pragma: keep

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

extern pros::Controller master;
extern rd::Console console;

static std::unique_ptr<pros::MotorGroup> lift;
static std::unique_ptr<pros::Rotation> lift_rotation;
static std::unique_ptr<pros::adi::DigitalOut> claw_open;
static std::unique_ptr<pros::adi::DigitalOut> claw_down;
static bool claw_opened = false;
static bool claw_downed = false;
static bool lift_rotation_zeroed = false;

void initialize_systems(int liftLeftPort, int liftRightPort, int liftRotationPort, char clawOpenPort, char clawDownPort) {
    if (liftLeftPort == liftRightPort) {
        lift = std::make_unique<pros::MotorGroup>(
            std::initializer_list<std::int8_t>{static_cast<std::int8_t>(liftLeftPort)},
            pros::MotorGears::red
        );
    } else {
        lift = std::make_unique<pros::MotorGroup>(
            std::initializer_list<std::int8_t>{
            static_cast<std::int8_t>(liftLeftPort),
            static_cast<std::int8_t>(liftRightPort)
            },
            pros::MotorGears::red
        );
    }
    lift->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift_rotation = std::make_unique<pros::Rotation>(static_cast<std::int8_t>(liftRotationPort));
    claw_open = std::make_unique<pros::adi::DigitalOut>(clawOpenPort, false);
    claw_down = std::make_unique<pros::adi::DigitalOut>(clawDownPort, false);
}

bool reset_lift_rotation() {
    lift_rotation_zeroed = lift_rotation && lift_rotation->reset_position() != PROS_ERR;
    return lift_rotation_zeroed;
}

bool move_lift_to_height(double heightInches) {
    if (!lift || !lift_rotation) {
        console.println("lift move failed: systems are not initialized");
        return false;
    }
    if (!lift_rotation_zeroed) {
        console.println("lift move failed: rotation sensor was not zeroed");
        return false;
    }
    if (!std::isfinite(heightInches) || heightInches < 6.5 || heightInches > 36.0) {
        console.println("lift move failed: height must be between 6.5 and 36 inches");
        return false;
    }

    struct CalibrationPoint {
        double height;
        double angle;
    };
    constexpr CalibrationPoint calibration[] = {
        {6.5, 0.0},
        {13.0, -25.0},
        {20.75, -42.0},
        {36.0, -83.0}
    };

    double targetAngle = calibration[0].angle;
    for (std::size_t i = 1; i < std::size(calibration); ++i) {
        if (heightInches <= calibration[i].height) {
            const auto& lower = calibration[i - 1];
            const auto& upper = calibration[i];
            const double fraction = (heightInches - lower.height) / (upper.height - lower.height);
            targetAngle = lower.angle + fraction * (upper.angle - lower.angle);
            break;
        }
    }

    constexpr float kP = 4.0;
    constexpr float kI = 0.0;
    constexpr float kD = 0.12;
    constexpr double positionTolerance = 3;
    constexpr uint32_t timeoutMs = 6000;

    lemlib::PID liftPid(kP, kI, kD);
    liftPid.reset();

    uint32_t startTime = pros::millis();
    uint32_t settledSince = 0;

    while (pros::millis() - startTime < timeoutMs) {
        const int32_t position = lift_rotation->get_position();
        const int32_t velocity = lift_rotation->get_velocity();
        if (position == PROS_ERR || velocity == PROS_ERR) {
            lift->move(0);
            console.println("lift move failed: rotation sensor read error");
            return false;
        }

        const uint32_t now = pros::millis();
        const double currentAngle = position / 100.0;
        const double error = targetAngle - currentAngle;
        const double output = -liftPid.update(error);

        if (lift->move(static_cast<std::int32_t>(std::clamp(output, -127.0, 127.0))) == PROS_ERR) {
            lift->move(0);
            console.println("lift move failed: motor command error");
            return false;
        }

        if (std::abs(velocity) <= 500 && std::abs(error) <= positionTolerance) {
            if (settledSince == 0) settledSince = now;
            if (now - settledSince >= 100) {
                lift->move(0);
                return true;
            }
        } else {
            settledSince = 0;
        }

        pros::delay(10);
    }

    lift->move(0);
    console.println("lift move failed: timed out; rotation=" + std::to_string(lift_rotation->get_position()));
    return false;
}

void lift_opcontrol() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        lift->move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        lift->move(-127);
    } else {
        lift->move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        claw_opened = !claw_opened;
        claw_open->set_value(claw_opened);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        claw_downed = !claw_downed;
        claw_down->set_value(claw_downed);
    }
}

void run_claw_test() {
    claw_open->set_value(false);
    pros::delay(1000);
    claw_open->set_value(true);
}
