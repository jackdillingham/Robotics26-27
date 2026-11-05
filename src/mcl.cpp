#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

// Helpers used only inside this file.
namespace {

constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;

// Pick the shortest signed angle difference. For example, 359 -> 1 is +2
// degrees, instead of -358. This also keeps particle headings near zero.
double wrap_heading(double angle) {
    return std::remainder(angle, 360.0);
}

// NaN/infinity can spread through every particle, so reject them up front.
bool pose_is_finite(lemlib::Pose pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

}

namespace localization {

///
// SETUP AND BACKGROUND TASK
///

MCL::MCL(lemlib::Chassis& chassis, std::vector<DistanceSensor> sensors, Field field, Config config)
    : chassis(chassis), sensors(std::move(sensors)), field(field), config(config) {
    // Refuse settings that make the math undefined or use unreasonable resources.
    // Nothing starts in this constructor: initialize() must calibrate LemLib first.
    valid = std::isfinite(field.width) && field.width > 0 && std::isfinite(field.height) &&
            field.height > 0 && std::isfinite(field.centerX) && std::isfinite(field.centerY) &&
            this->sensors.size() >= 2 && config.particles >= 50 && config.particles <= 2000 &&
            config.periodMs >= 20 && config.measurementStd > 0 && config.minRange > 0 &&
            config.maxRange > config.minRange && config.initialPositionStd > 0 &&
            config.initialHeadingStd > 0 && config.motionPositionStd >= 0 &&
            config.motionHeadingStd >= 0 && config.inlierThreshold > 0 &&
            config.maxPositionStd > 0 && config.maxHeadingStd > 0 && config.correctionGain > 0 &&
            config.correctionGain <= 1 && config.maxPositionCorrection > 0 &&
            config.maxHeadingCorrection > 0;
    // Positive checks alone do not reject +infinity, so check every setting too.
    const double values[] = {
        config.measurementStd,      config.minRange,          config.maxRange,
        config.initialPositionStd,  config.initialHeadingStd, config.motionPositionStd,
        config.motionHeadingStd,    config.inlierThreshold,   config.maxPositionStd,
        config.maxHeadingStd,       config.correctionGain,    config.maxPositionCorrection,
        config.maxHeadingCorrection};
    for (double value : values) {
        valid = valid && std::isfinite(value);
    }
    for (const auto& sensor : this->sensors) {
        valid = valid && sensor.sensor && std::isfinite(sensor.forward) &&
                std::isfinite(sensor.right) && std::isfinite(sensor.heading);
    }
    // Allocate once, instead of allocating these buffers every 50 ms.
    if (valid) {
        particles.resize(config.particles);
        resampledParticles.resize(config.particles);
        readings.resize(this->sensors.size());
    }
}

MCL::~MCL() {
    disable();
    if (task) {
        task->remove();
    }
}

void MCL::start() {
    std::lock_guard<pros::Mutex> lock(mutex);
    if (!valid || task) {
        return;
    }
    // [this] lets the task use this MCL object's sensors, particles, and settings.
    // The task stays alive while disabled; it just skips update(). This avoids
    // creating a new task every time competition switches modes.
    task.reset(new pros::Task(
        [this] {
            while (true) {
                {
                    std::lock_guard<pros::Mutex> lock(mutex);
                    if (enabled) {
                        update();
                    }
                }
                // Sleep OUTSIDE the lock, so enable/disable/setPose can run meanwhile.
                pros::delay(config.periodMs);
            }
        },
        "MCL"));
}

void MCL::enable() {
    std::lock_guard<pros::Mutex> lock(mutex);
    const auto pose = chassis.getPose();
    if (!valid || !task || !pose_is_finite(pose) || !inside(pose.x, pose.y)) {
        return;
    }
    // Do not count movement made while disabled as one giant odometry step.
    seed(pose);
    enabled = true;
}

void MCL::disable() {
    std::lock_guard<pros::Mutex> lock(mutex);
    enabled = false;
}

bool MCL::isEnabled() {
    std::lock_guard<pros::Mutex> lock(mutex);
    return enabled;
}

void MCL::setPose(lemlib::Pose pose) {
    std::lock_guard<pros::Mutex> lock(mutex);
    if (!pose_is_finite(pose)) {
        return;
    }
    // Reset LemLib and the particles under the same lock. Calling chassis.setPose
    // directly during an update can otherwise look like actual robot movement.
    chassis.setPose(pose);
    if (valid && inside(pose.x, pose.y)) {
        seed(pose);
    } else {
        enabled = false;
    }
}

///
// PARTICLE AND WALL HELPERS
///

// Draw a bell-curve random error. Small errors are common, large errors rare.
// Zero spread means no error; do not create a normal distribution with zero std.
double MCL::noise(double standardDeviation) {
    if (standardDeviation <= 0) {
        return 0;
    }
    return std::normal_distribution<double>(0, standardDeviation)(random);
}

bool MCL::inside(double x, double y) const {
    return x >= field.centerX - field.width / 2 && x <= field.centerX + field.width / 2 &&
           y >= field.centerY - field.height / 2 && y <= field.centerY + field.height / 2;
}

void MCL::seed(lemlib::Pose pose) {
    // Start with equal trust in every guess, scattered near the supplied pose.
    // Clamp positions to the field so none begin outside the walls.
    previous = pose;
    for (auto& particle : particles) {
        particle = {std::clamp(pose.x + noise(config.initialPositionStd),
                               field.centerX - field.width / 2, field.centerX + field.width / 2),
                    std::clamp(pose.y + noise(config.initialPositionStd),
                               field.centerY - field.height / 2, field.centerY + field.height / 2),
                    wrap_heading(pose.theta + noise(config.initialHeadingStd)),
                    1.0 / particles.size()};
    }
}

double MCL::expected(const Particle& particle, const DistanceSensor& sensor) const {
    // Move the sensor lens from robot coordinates into field coordinates.
    // At heading 0: forward adds to Y, and right adds to X.
    // At heading 90: forward adds to X, and right subtracts from Y.
    const double robotHeading = particle.theta * DEGREES_TO_RADIANS;
    const double sensorX = particle.x + sensor.forward * std::sin(robotHeading) +
                           sensor.right * std::cos(robotHeading);
    const double sensorY = particle.y + sensor.forward * std::cos(robotHeading) -
                           sensor.right * std::sin(robotHeading);
    if (!inside(sensorX, sensorY)) {
        return std::numeric_limits<double>::infinity();
    }

    // Send an imaginary beam from the lens along the sensor's facing direction.
    // The direction vector has length 1, so beam travel is distance in inches.
    const double directionX = std::sin((particle.theta + sensor.heading) * DEGREES_TO_RADIANS);
    const double directionY = std::cos((particle.theta + sensor.heading) * DEGREES_TO_RADIANS);
    double distance = std::numeric_limits<double>::infinity();

    // Find where the beam reaches an X wall and a Y wall; the nearer one wins.
    // Skip almost-zero components to avoid dividing by zero for parallel beams.
    if (std::abs(directionX) > 1e-9) {
        double wallX = field.centerX - field.width / 2; // Left wall.
        if (directionX > 0) {
            wallX = field.centerX + field.width / 2; // Right wall.
        }
        distance = (wallX - sensorX) / directionX;
    }

    if (std::abs(directionY) > 1e-9) {
        double wallY = field.centerY - field.height / 2; // Bottom wall.
        if (directionY > 0) {
            wallY = field.centerY + field.height / 2; // Top wall.
        }
        const double distanceToYWall = (wallY - sensorY) / directionY;
        distance = std::min(distance, distanceToYWall);
    }
    return distance;
}

///
// MAIN MCL LOOP
///

void MCL::update() {
    // 1. READ HOW FAR LEMLIB SAYS WE MOVED
    // Use the change since the last update, not the absolute odometry position.
    const auto current = chassis.getPose();
    if (!pose_is_finite(current)) {
        return;
    }
    const double deltaX = current.x - previous.x;
    const double deltaY = current.y - previous.y;
    const double deltaHeading = wrap_heading(current.theta - previous.theta);
    // More than 12 inches or 45 degrees in one update is treated as a reset.
    // This is a fallback, not a replacement for using mcl.setPose(). If you slow
    // periodMs a lot, normal movement could exceed these per-update limits.
    if (std::hypot(deltaX, deltaY) > 12 || std::abs(deltaHeading) > 45) {
        seed(current);
        return;
    }
    // Convert field X/Y movement to robot forward/right movement. Each particle
    // then applies that same movement using its OWN guessed heading.
    const double previousHeading = previous.theta * DEGREES_TO_RADIANS;
    const double forward = deltaX * std::sin(previousHeading) + deltaY * std::cos(previousHeading);
    const double right = deltaX * std::cos(previousHeading) - deltaY * std::sin(previousHeading);
    previous = current;
    const double positionNoise = config.motionPositionStd * std::sqrt(std::hypot(deltaX, deltaY));
    const double headingNoise = config.motionHeadingStd * std::sqrt(std::abs(deltaHeading));

    // 2. MOVE EVERY GUESS
    // Add random motion error so guesses can account for slip and odometry drift.
    // With no movement these motion-noise terms are zero.
    for (auto& particle : particles) {
        const double particleHeading = particle.theta * DEGREES_TO_RADIANS;
        particle.x += forward * std::sin(particleHeading) + right * std::cos(particleHeading) +
                      noise(positionNoise);
        particle.y += forward * std::cos(particleHeading) - right * std::sin(particleHeading) +
                      noise(positionNoise);
        particle.theta = wrap_heading(particle.theta + deltaHeading + noise(headingNoise));
    }

    // 3. READ THE DISTANCE SENSORS
    // PROS returns millimeters. Convert once; the rest of this file uses inches.
    // 9999 means no object detected, PROS_ERR means a failed sensor read.
    // Store -1 for readings we should ignore.
    unsigned usableReadings = 0;
    for (unsigned i = 0; i < sensors.size(); ++i) {
        const auto millimeters = sensors[i].sensor->get();
        const double inches = millimeters / 25.4;
        readings[i] = (millimeters != PROS_ERR && millimeters != 9999 &&
                       inches >= config.minRange && inches <= config.maxRange)
                          ? inches
                          : -1;
        if (readings[i] >= 0) {
            ++usableReadings;
        }
    }
    if (usableReadings < 2) {
        return; // Odometry still runs; only the wall correction is skipped.
    }

    // 4. SCORE THE GUESSES AGAINST THE WALLS
    // Predict each sensor reading at each guessed pose. A smaller difference
    // gets a higher likelihood, which increases that particle's relative trust.
    double totalWeight = 0;
    for (auto& particle : particles) {
        double likelihood = inside(particle.x, particle.y) ? 1 : 0;
        for (unsigned i = 0; i < sensors.size(); ++i) {
            if (readings[i] < 0) {
                continue;
            }
            const double normalizedError =
                (readings[i] - expected(particle, sensors[i])) / config.measurementStd;
            // exp(-0.5 * error^2) is 1 for a perfect match and falls toward 0
            // for a bad match. Keep a 5% floor so one blocked sensor does not
            // completely eliminate a good particle. This cannot reject every
            // obstacle; the agreement checks below still have to pass.
            likelihood *= 0.05 + 0.95 * std::exp(-0.5 * normalizedError * normalizedError);
        }
        particle.weight *= likelihood;
        totalWeight += particle.weight;
    }
    // If all weights vanish (for example, every guess left the field), restart
    // around odometry instead of dividing by zero and spreading invalid values.
    if (!(totalWeight > 0) || !std::isfinite(totalWeight)) {
        seed(current);
        return;
    }

    // 5. CALCULATE THE WEIGHTED AVERAGE POSE
    // Divide weights by their sum so they add up to 1.
    // Average heading with sine/cosine: the mean of 359 and 1 should be 0, not 180.
    double estimatedX = 0;
    double estimatedY = 0;
    double headingSin = 0;
    double headingCos = 0;
    for (auto& particle : particles) {
        particle.weight /= totalWeight;
        estimatedX += particle.weight * particle.x;
        estimatedY += particle.weight * particle.y;
        headingSin += particle.weight * std::sin(particle.theta * DEGREES_TO_RADIANS);
        headingCos += particle.weight * std::cos(particle.theta * DEGREES_TO_RADIANS);
    }
    const double estimatedHeading = std::atan2(headingSin, headingCos) / DEGREES_TO_RADIANS;

    // Measure how tightly the guesses agree. Variance is squared spread;
    // sqrt(variance) below converts it back to inches or degrees.
    double positionVariance = 0;
    double headingVariance = 0;
    double sumSquaredWeights = 0;
    for (const auto& particle : particles) {
        positionVariance +=
            particle.weight * ((particle.x - estimatedX) * (particle.x - estimatedX) +
                               (particle.y - estimatedY) * (particle.y - estimatedY));
        headingVariance +=
            particle.weight * std::pow(wrap_heading(particle.theta - estimatedHeading), 2);
        sumSquaredWeights += particle.weight * particle.weight;
    }

    // 6. DECIDE WHETHER THE ESTIMATE IS GOOD ENOUGH TO USE
    // A tight cluster is not sufficient: actual readings must also agree with
    // the walls predicted at the average pose.
    unsigned agreeingSensors = 0;
    // Require sensor beams pointing in different directions. Front + back alone
    // are opposite but parallel, so they cannot independently check both axes.
    // abs(sin(angle difference)) > 0.5 requires more than 30 degrees separation
    // from parallel. This is a simple geometry check, not proof of a unique pose.
    double firstDirection = 0;
    bool hasDifferentDirections = false;
    const Particle averagePose{estimatedX, estimatedY, estimatedHeading, 1};
    for (unsigned i = 0; i < sensors.size(); ++i) {
        if (readings[i] < 0) {
            continue;
        }

        const double predictedDistance = expected(averagePose, sensors[i]);
        if (std::abs(readings[i] - predictedDistance) > config.inlierThreshold) {
            continue;
        }

        if (agreeingSensors == 0) {
            firstDirection = sensors[i].heading;
        } else {
            const double angleDifference =
                (sensors[i].heading - firstDirection) * DEGREES_TO_RADIANS;
            if (std::abs(std::sin(angleDifference)) > 0.5) {
                hasDifferentDirections = true;
            }
        }
        ++agreeingSensors;
    }

    if (agreeingSensors >= 2 && hasDifferentDirections && inside(estimatedX, estimatedY) &&
        std::sqrt(positionVariance) <= config.maxPositionStd &&
        std::sqrt(headingVariance) <= config.maxHeadingStd) {
        // 7. FEED A SMALL CORRECTION BACK INTO LEMLIB
        // Apply only correctionGain of the error, then cap its size.
        // Read odometry again so movement during this calculation is preserved.
        auto latest = chassis.getPose();
        if (pose_is_finite(latest)) {
            double correctionX = (estimatedX - current.x) * config.correctionGain;
            double correctionY = (estimatedY - current.y) * config.correctionGain;
            const double correctionDistance = std::hypot(correctionX, correctionY);
            // Scale both axes together to keep the correction direction.
            if (correctionDistance > config.maxPositionCorrection) {
                correctionX *= config.maxPositionCorrection / correctionDistance;
                correctionY *= config.maxPositionCorrection / correctionDistance;
            }
            const double correctionHeading =
                std::clamp(wrap_heading(estimatedHeading - current.theta) * config.correctionGain,
                           -config.maxHeadingCorrection, config.maxHeadingCorrection);
            chassis.setPose(latest.x + correctionX, latest.y + correctionY,
                            latest.theta + correctionHeading);
            // IMPORTANT: include our correction in the previous odometry pose,
            // or next update would count our own setPose() as robot movement.
            // Use 'current' here, not 'latest': particles have only been moved up
            // to 'current'. Movement since then belongs in the NEXT update.
            previous = {static_cast<float>(current.x + correctionX),
                        static_cast<float>(current.y + correctionY),
                        static_cast<float>(current.theta + correctionHeading)};
        }
    }

    // 8. REPLACE UNLIKELY GUESSES WITH COPIES OF LIKELY ONES
    // 1 / sum(weight^2) estimates how many particles meaningfully contribute.
    // Resample when that drops below half the particle count.
    // Walk through the cumulative weights at evenly spaced sample points.
    // A high-weight particle covers more of that line, so it gets more copies.
    if (sumSquaredWeights > 2.0 / particles.size()) {
        const double equalWeight = 1.0 / particles.size();
        double samplePoint = std::uniform_real_distribution<double>(0, equalWeight)(random);
        double cumulativeWeight = particles[0].weight;
        unsigned particleIndex = 0;
        for (auto& particle : resampledParticles) {
            while (samplePoint > cumulativeWeight && particleIndex + 1 < particles.size()) {
                ++particleIndex;
                cumulativeWeight += particles[particleIndex].weight;
            }
            particle = particles[particleIndex];
            particle.weight = equalWeight;
            // Add tiny differences to the copies so they do not all become
            // identical. This is called roughening (0.03 inches/degrees here).
            particle.x += noise(0.03);
            particle.y += noise(0.03);
            particle.theta = wrap_heading(particle.theta + noise(0.03));
            samplePoint += equalWeight;
        }
        particles.swap(resampledParticles);
    }
}
} // namespace localization
