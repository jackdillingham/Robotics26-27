#ifndef _LOCALIZATION_HPP_
#define _LOCALIZATION_HPP_

#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include <vector> // IWYU pragma: keep
#include <cmath> // IWYU pragma: keep
#include <random> // IWYU pragma: keep


// ============================================================
// PORTS
// ============================================================
#define IMU_PORT 10

#define FWD_WHEEL_PORT 20
#define SIDE_WHEEL_PORT 19

#define PORT_FRONT 1
#define PORT_LEFT 2
#define PORT_BACK 3
#define PORT_RIGHT 4



// ============================================================
// CONSTANTS
// ============================================================
const double FIELD_WIDTH = 144.0;   // inches
const double FIELD_HEIGHT = 144.0;  // inches

const double WHEEL_DIAMETER = 2.75;  // inches
const double TICKS_PER_REV = 360.0;
const double DEG_TO_INCH = (M_PI * WHEEL_DIAMETER) / 360.0;

// Tracking wheel offsets from robot center (inches)
const double FWD_WHEEL_OFFSET = 0.0;  // along side axis
const double SIDE_WHEEL_OFFSET = 0.0; // along forward axis

// Distance sensor offsets from robot center (inches)
struct SensorConfig {
    double sx, sy;           // offset from robot center (robot frame)
    double mountingAngle;    // radians relative to robot heading
    int port;                // smart port
};

const SensorConfig SENSORS[4] = {
    // Front: pointing forward (0 rad), mounted d_front inches forward
    {+6.0,  0.0,  0.0,          PORT_FRONT},
    // Left: pointing left (+π/2), mounted d_left inches left
    { 0.0, +6.0,  M_PI / 2,     PORT_LEFT},
    // Back: pointing backward (π), mounted d_back inches behind
    {-6.0,  0.0,  M_PI,         PORT_BACK},
    // Right: pointing right (-π/2), mounted d_right inches right
    { 0.0, -6.0, -M_PI / 2,     PORT_RIGHT},
};

// Fusion parameters
const double BLEND_WEIGHT = 0.3;       // correction blend factor
const double MAX_RESIDUAL = 6.0;       // inches — reject readings off by more than this
const double CROSS_CHECK_THRESHOLD = 3.0; // inches
const int FILTER_SAMPLES = 5;          // averaging samples for distance readings

// MCL parameters (if using particle filter)
const int NUM_PARTICLES = 300;
const double SIGMA_XY = 0.5;
const double SIGMA_THETA = 0.02;
const double SIGMA_SENSOR = 1.5;



#endif // _LOCALIZATION_HPP_