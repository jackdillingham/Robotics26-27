#include "main.h" // IWYU pragma: keep

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>

extern pros::Controller master;
extern rd::Console console;

static std::unique_ptr<pros::MotorGroup> lift_motors;
static std::unique_ptr<pros::Rotation> lift_rotation;
static std::unique_ptr<pros::adi::DigitalOut> claw_open;
static std::unique_ptr<pros::adi::DigitalOut> claw_down;
static bool claw_opened = false;
static bool claw_downed = false;
static bool lift_rotation_zeroed = false;
static bool lift_manual_active = false;
static std::atomic<bool> lift_move_active{false};
static pros::Mutex claw_mutex;

namespace lift {

static bool set_claw_up_and_closed() {
    if (!claw_mutex.take()) {
        console.println("claw update failed: could not lock claw controls");
        return false;
    }

    if (!claw_open || !claw_down) {
        claw_mutex.give();
        return false;
    }

    claw_opened = false;
    claw_downed = false;
    claw_open->set_value(false);
    claw_down->set_value(false);
    claw_mutex.give();
    return true;
}

void initialize_systems(int liftLeftPort, int liftRightPort, int liftRotationPort, char clawOpenPort, char clawDownPort) {
    if (liftLeftPort == liftRightPort) {
        lift_motors = std::make_unique<pros::MotorGroup>(
            std::initializer_list<std::int8_t>{static_cast<std::int8_t>(liftLeftPort)},
            pros::MotorGears::red
        );
    } else {
        lift_motors = std::make_unique<pros::MotorGroup>(
            std::initializer_list<std::int8_t>{
            static_cast<std::int8_t>(liftLeftPort),
            static_cast<std::int8_t>(liftRightPort)
            },
            pros::MotorGears::red
        );
    }
    lift_motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift_rotation = std::make_unique<pros::Rotation>(static_cast<std::int8_t>(liftRotationPort));
    claw_open = std::make_unique<pros::adi::DigitalOut>(clawOpenPort, false);
    claw_down = std::make_unique<pros::adi::DigitalOut>(clawDownPort, false);
}

bool reset_rotation() {
    lift_rotation_zeroed = lift_rotation && lift_rotation->reset_position() != PROS_ERR;
    return lift_rotation_zeroed;
}

void hold() {
    if (!lift_motors) {
        return;
    }

    lift_motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    const double position = lift_motors->get_position();
    if (!std::isfinite(position) || lift_motors->move_absolute(position, 100) == PROS_ERR) {
        lift_motors->move(0);
        console.println("lift hold failed: motor encoder or command error");
    }
    lift_manual_active = false;
}

void coast() {
    if (lift_motors) {
        lift_motors->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift_motors->move(0);
    }
    lift_manual_active = false;
}

static bool run_to_angle(double targetAngle, int maxSpeed) {
    if (!set_claw_up_and_closed()) {
        hold();
        console.println("lift move failed: could not set claw up and closed");
        return false;
    }

    constexpr float kP = 7.0;
    constexpr float kI = 0.0;
    constexpr float kD = 3.0;
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
            hold();
            console.println("lift move failed: rotation sensor read error");
            return false;
        }

        const uint32_t now = pros::millis();
        const double currentAngle = position / 100.0;
        const double error = targetAngle - currentAngle;
        const double output = -liftPid.update(error);

        if (lift_motors->move(static_cast<std::int32_t>(
                std::clamp(output, -static_cast<double>(maxSpeed), static_cast<double>(maxSpeed)))) == PROS_ERR) {
            hold();
            console.println("lift move failed: motor command error");
            return false;
        }

        if (std::abs(velocity) <= 500 && std::abs(error) <= positionTolerance) {
            if (settledSince == 0) settledSince = now;
            if (now - settledSince >= 100) {
                hold();
                console.println(
                    "lift reached angle=" + std::to_string(currentAngle) +
                    " target=" + std::to_string(targetAngle)
                );
                return true;
            }
        } else {
            settledSince = 0;
        }

        pros::delay(10);
    }

    hold();
    console.println(
        "lift move timed out: angle=" + std::to_string(lift_rotation->get_position() / 100.0) +
        " target=" + std::to_string(targetAngle)
    );
    return false;
}

bool move_to_height(double heightInches) {
    return move_to_height(heightInches, 127);
}

bool move_to_height(double heightInches, int maxSpeed) {
    if (!lift_motors || !lift_rotation) {
        console.println("lift move failed: systems are not initialized");
        return false;
    }
    if (!lift_rotation_zeroed) {
        console.println("lift move failed: rotation sensor was not zeroed");
        return false;
    }
    if (!std::isfinite(heightInches) || heightInches < 6.5 || heightInches > 32.0) {
        console.println("lift move failed: height must be between 6.5 and 36 inches");
        return false;
    }
    if (maxSpeed < 1 || maxSpeed > 127) {
        console.println("lift move failed: max speed must be between 1 and 127");
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

    bool expectedIdle = false;
    if (!lift_move_active.compare_exchange_strong(expectedIdle, true)) {
        console.println("lift move not started: another move is active");
        return false;
    }

    if (!set_claw_up_and_closed()) {
        lift_move_active.store(false);
        return false;
    }
    const pros::task_t task = pros::Task::create([targetAngle, maxSpeed]() {
        run_to_angle(targetAngle, maxSpeed);
        lift_move_active.store(false);
    }, "Lift PID");

    if (task == nullptr) {
        lift_move_active.store(false);
        console.println("lift move failed: could not create PID task");
        return false;
    }

    return true;
}

void wait_until_done() {
    while (lift_move_active.load()) {
        pros::delay(10);
    }
}

void open() {
    if (!claw_mutex.take()) {
        console.println("claw open failed: could not lock claw controls");
        return;
    }
    if (!lift_move_active.load() && claw_open) {
        claw_opened = true;
        claw_open->set_value(true);
    }
    claw_mutex.give();
}

void close() {
    if (!claw_mutex.take()) {
        console.println("claw close failed: could not lock claw controls");
        return;
    }
    if (!lift_move_active.load() && claw_open) {
        claw_opened = false;
        claw_open->set_value(false);
    }
    claw_mutex.give();
}

void up() {
    if (!lift_motors || lift_move_active.load()) {
        return;
    }
    lift_motors->move(127);
    lift_manual_active = true;
}

void down() {
    if (!lift_motors || lift_move_active.load()) {
        return;
    }
    lift_motors->move(-127);
    lift_manual_active = true;
}

void toggle_claw_up_down() {
    if (!claw_mutex.take()) {
        console.println("claw toggle failed: could not lock claw controls");
        return;
    }
    if (lift_move_active.load() || !claw_down) {
        claw_mutex.give();
        return;
    }

    claw_downed = !claw_downed;
    claw_down->set_value(claw_downed);
    claw_mutex.give();
}

void toggle_claw_open_closed() {
    if (!claw_mutex.take()) {
        console.println("claw toggle failed: could not lock claw controls");
        return;
    }
    if (lift_move_active.load() || !claw_open) {
        claw_mutex.give();
        return;
    }

    claw_opened = !claw_opened;
    claw_open->set_value(claw_opened);
    claw_mutex.give();
}

void opcontrol() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        up();
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        down();
    } else {
        if (lift_manual_active) {
            hold();
        }
    }

    static bool previousA = false;
    static bool previousB = false;
    const bool buttonA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    const bool buttonB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (buttonA && !previousA) toggle_claw_open_closed();
    if (buttonB && !previousB) toggle_claw_up_down();
    previousA = buttonA;
    previousB = buttonB;
}

void run_claw_test() {
    if (!claw_mutex.take()) {
        console.println("claw test failed: could not lock claw controls");
        return;
    }
    if (lift_move_active.load() || !claw_open) {
        claw_mutex.give();
        return;
    }

    claw_open->set_value(false);
    claw_mutex.give();
    pros::delay(1000);
    if (!claw_mutex.take()) {
        console.println("claw test failed: could not lock claw controls");
        return;
    }
    if (!lift_move_active.load() && claw_open) {
        claw_open->set_value(true);
    }
    claw_mutex.give();
}

}
