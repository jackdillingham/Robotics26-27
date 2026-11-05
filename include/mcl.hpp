#ifndef MCL_HPP
#define MCL_HPP

#include "lemlib/api.hpp"
#include "pros/distance.hpp"

#include <memory>
#include <random>
#include <vector>

namespace localization {

///
// SENSOR AND FIELD SETUP
///

// All distances are in inches. All angles are in degrees.
// Like LemLib: 0 degrees faces +Y, and 90 degrees faces +X (clockwise).
struct DistanceSensor {
    pros::Distance* sensor; // Address of the sensor already created in main.cpp.
    double forward; // Odometry center to sensor lens: front positive, back negative.
    double right; // Odometry center to sensor lens: right positive, left negative.
    double heading; // Where it points relative to the bot: front 0, right 90, back 180.
};

// Use the inside wall measurements, in the SAME coordinate system as your autos.
// Example: {144, 144, 0, 0} puts the four walls at x/y = -72 and +72.
// Those example dimensions are placeholders, not measurements of Pinnacle.
struct Field {
    double width; // Distance between the -X and +X walls.
    double height; // Distance between the -Y and +Y walls.
    double centerX = 0;
    double centerY = 0;
};

///
// FILTER SETTINGS
///

// These are starting values to tune on the robot.
// "Std" means standard deviation: how widely guesses or errors are spread.
// A larger value means we allow more uncertainty.
struct Config {
    unsigned particles = 300; // Number of possible robot poses to keep track of.
    unsigned periodMs = 50; // Time between updates. Smaller = more CPU use.

    // Spread the guesses around the known starting pose when enabled or reset.
    double initialPositionStd = 2; // Inches.
    double initialHeadingStd = 2; // Degrees.

    // Add uncertainty as the robot moves, to account for wheel slip/odom error.
    // Per update: position noise = this value * sqrt(inches traveled).
    // Heading noise = this value * sqrt(abs(degrees turned)).
    double motionPositionStd = 0.08;
    double motionHeadingStd = 0.15;

    double measurementStd = 1.5; // Expected sensor error in inches; larger trusts it less.
    double minRange = 2; // Ignore readings closer than this many inches.
    double maxRange = 78; // Ignore readings farther than this many inches.

    // Only correct LemLib when the wall readings agree and the guesses cluster.
    double inlierThreshold = 4; // Maximum reading-vs-predicted-wall error in inches.
    double maxPositionStd = 2.5; // Maximum combined X/Y spread in inches.
    double maxHeadingStd = 4; // Maximum heading spread in degrees.

    // Example: gain 0.35 applies 35% of the estimated odometry error per update.
    // The limits below cap that correction so one update cannot make a big jump.
    double correctionGain = 0.35;
    double maxPositionCorrection = 1; // Inches per update, combined X/Y.
    double maxHeadingCorrection = 1; // Degrees per update.
};

///
// MONTE CARLO LOCALIZATION
///

// A particle is one guess of where the robot is. Each update moves the guesses
// using odometry, then favors the ones that match the distance sensor readings.
// The average of those guesses gives us a position to compare against LemLib.
//
// This tracks near a KNOWN starting pose. It cannot reliably find a robot that
// starts anywhere on the field. Its map contains only the four outside walls;
// game objects and other robots can block the readings.
//
// Setup in initialize(): chassis.calibrate(); mcl.start();
// At the start of an auto: mcl.setPose(startPose); mcl.enable();
// To stop corrections: mcl.disable();
// Keep this object alive for the whole robot program (like the global in main.cpp).
// Always use mcl.setPose() for intentional pose resets so both systems reset together.
//
// If corrections do not happen, check:
// 1. MCL_GEOMETRY_CONFIGURED in main.cpp and mcl.isEnabled().
// 2. Sensor ports, mounting offsets, field dimensions, and starting pose.
// 3. At least two sensors see walls in different directions, within min/maxRange.
// 4. The agreement/spread limits in Config. Opposite sensors alone are not enough.
// If corrections go the wrong way, check sensor signs/headings before tuning noise.
class MCL {
public:
    MCL(lemlib::Chassis& chassis, std::vector<DistanceSensor> sensors,
        Field field, Config config = {});
    ~MCL();

    // A second copy would share the chassis/task but have different particles.
    MCL(const MCL&) = delete;
    MCL& operator=(const MCL&) = delete;

    void start(); // Create the background task once. Does not enable corrections.
    void enable(); // Restart guesses around the current chassis pose.
    void disable(); // Wait for any current update, then stop further corrections.
    bool isEnabled(); // False if disabled, or setup/start pose prevented enabling.
    void setPose(lemlib::Pose pose); // Set LemLib's pose AND reset the guesses together.

private:
    struct Particle {
        double x; // Guessed field X, in inches.
        double y; // Guessed field Y, in inches.
        double theta; // Guessed heading, in degrees.
        double weight; // Relative trust in this guess; all weights add up to 1.
    };

    lemlib::Chassis& chassis;
    std::vector<DistanceSensor> sensors;
    Field field;
    Config config;

    // Only one task can update/reset the filter at a time. The lock is released
    // automatically at the end of each function/block that uses lock_guard.
    pros::Mutex mutex;
    std::unique_ptr<pros::Task> task;
    bool enabled = false;
    bool valid = false; // Constructor settings passed the checks in mcl.cpp.

    std::vector<Particle> particles;
    std::vector<Particle> resampledParticles; // Reusable buffer for copying good guesses.
    std::vector<double> readings; // Inches; -1 means ignore this sensor this update.
    lemlib::Pose previous{0, 0, 0}; // Last odometry baseline, including our corrections.
    std::mt19937 random{0x4d434c}; // Fixed random seed makes test runs repeatable.

    double noise(double standardDeviation); // Random error centered on zero.
    void seed(lemlib::Pose pose); // Create a fresh set of guesses around this pose.
    void update(); // One complete filter cycle; called by the background task.
    double expected(const Particle& particle, const DistanceSensor& sensor) const;
    bool inside(double x, double y) const; // Is this field point within the walls?
};

}

extern localization::MCL mcl; // Created with the four sensors in main.cpp.

#endif
