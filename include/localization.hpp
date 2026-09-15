#ifndef _LOCALIZATION_HPP_
#define _LOCALIZATION_HPP_

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include <vector>
#include <cmath>
#include <random>

#define PI 3.14159265358979323846

// ============================================================
// CONSTANTS — TUNE THESE
// ============================================================
const double FIELD_WIDTH = 144.0;
const double FIELD_HEIGHT = 144.0;

struct SensorConfig {
    double sx, sy;
    double mountingAngle;
};

// Order: front, back, left, right — keep consistent everywhere
const SensorConfig SENSORS[4] = {
    {+6.0,  0.0,  0.0,        },
    {-6.0,  0.0,  PI,       },
    { 0.0, +6.0,  PI / 2,   },
    { 0.0, -6.0, -PI / 2,   },
};

const int NUM_PARTICLES = 300;
const double SIGMA_XY = 0.5;       // inches — particle motion noise
const double SIGMA_THETA = 0.02;   // radians — particle heading noise
const double SIGMA_SENSOR = 1.5;   // inches — sensor measurement noise
const int WALL_SIZE_THRESHOLD = 50; // tune against your field's wall vs. game-object sizes

#endif // _LOCALIZATION_HPP_