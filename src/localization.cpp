#include "localization.hpp" // IWYU pragma: keep


// ============================================================
// POSE / PARTICLE
// ============================================================
struct Pose { double x, y, theta; };
struct Particle { double x, y, theta, weight; };

std::vector<Particle> particles;
std::mt19937 rng(42);
std::normal_distribution<double> noiseDist(0.0, 1.0);
std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

double gaussianNoise(double mean, double stddev) {
    return mean + noiseDist(rng) * stddev;
}

double wrapAngle(double a) {
    while (a >  PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

// Bridge between LemLib's own Pose type and this guide's local Pose struct:
// always read LemLib's pose in radians + standard-position form so the
// raycast/MCL math matches without manual conversion (see Section 2).
Pose getLemlibPoseStandard(lemlib::Chassis& chassis) {
    lemlib::Pose p = chassis.getPose(true, true);
    return {p.x, p.y, p.theta};
}

void initMCL(double startX, double startY, double startTheta) {
    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({
            startX + gaussianNoise(0, 1.0),
            startY + gaussianNoise(0, 1.0),
            wrapAngle(startTheta + gaussianNoise(0, 0.05)),
            1.0 / NUM_PARTICLES
        });
    }
}

// ============================================================
// RAYCAST — expected distance from a pose to the nearest wall
// ============================================================
double raycastToWall(double x, double y, double theta, const SensorConfig& sensor) {
    double cosT = cos(theta), sinT = sin(theta);
    double psx = x + (sensor.sx * cosT - sensor.sy * sinT);
    double psy = y + (sensor.sx * sinT + sensor.sy * cosT);

    double phi = theta + sensor.mountingAngle;
    double dx = cos(phi), dy = sin(phi);

    double minDist = 1e9;
    if (dx < -0.001) { double t = -psx / dx; if (t > 0 && t < minDist) minDist = t; }
    if (dx >  0.001) { double t = (FIELD_WIDTH - psx) / dx; if (t > 0 && t < minDist) minDist = t; }
    if (dy < -0.001) { double t = -psy / dy; if (t > 0 && t < minDist) minDist = t; }
    if (dy >  0.001) { double t = (FIELD_HEIGHT - psy) / dy; if (t > 0 && t < minDist) minDist = t; }
    return minDist;
}

double gaussianProb(double actual, double expected, double sigma) {
    double diff = actual - expected;
    return exp(-0.5 * (diff * diff) / (sigma * sigma));
}

// ============================================================
// DISTANCE SENSOR READ + VALIDATE
// ============================================================
// Returns inches, or -1 if the reading should be skipped this cycle.
double readValidatedDistance(pros::Distance& sensor) {
    double mm = sensor.get();
    int confidence = sensor.get_confidence();
    int objectSize = sensor.get_object_size();

    if (mm < 20 or mm > 2000) return -1;
    if (mm > 200 and confidence < 20) return -1;
    if (objectSize < WALL_SIZE_THRESHOLD) return -1;

    return mm / 25.4;
}

// ============================================================
// MCL UPDATE
// ============================================================
// localForward/localStrafe: the robot's motion since last tick, in the
// ROBOT's own local frame (not field frame) — see computeLocalDelta() below.
// dtheta: heading change since last tick (radians).
void mclPredict(double localForward, double localStrafe, double dtheta) {
    for (auto& p : particles) {
        double pMid = p.theta + dtheta / 2.0;
        double motionX = localForward * cos(pMid) - localStrafe * sin(pMid);
        double motionY = localForward * sin(pMid) + localStrafe * cos(pMid);

        p.x     += motionX + gaussianNoise(0, SIGMA_XY);
        p.y     += motionY + gaussianNoise(0, SIGMA_XY);
        p.theta  = wrapAngle(p.theta + dtheta + gaussianNoise(0, SIGMA_THETA));
    }
}

void mclWeightAndResample(double sensorReadings[4]) {
    // If no sensor reading is valid this cycle, skip correction entirely —
    // resampling with nothing to weight against just injects noise for no gain.
    int validCount = 0;
    for (int i = 0; i < 4; i++) {
        if (sensorReadings[i] >= 0) validCount++;
    }
    if (validCount == 0) return;

    // --- WEIGHT ---
    double totalWeight = 0;
    for (auto& p : particles) {
        p.weight = 1.0;
        for (int i = 0; i < 4; i++) {
            if (sensorReadings[i] < 0) continue;  // invalid, skip this sensor
            double expected = raycastToWall(p.x, p.y, p.theta, SENSORS[i]);
            p.weight *= gaussianProb(sensorReadings[i], expected, SIGMA_SENSOR);
        }
        totalWeight += p.weight;
    }

    if (totalWeight <= 0) {
        for (auto& p : particles) p.weight = 1.0 / NUM_PARTICLES;
        return;  // no usable signal this cycle — keep particles as-is
    }
    for (auto& p : particles) p.weight /= totalWeight;

    // --- SYSTEMATIC RESAMPLING ---
    std::vector<Particle> newParticles;
    newParticles.reserve(NUM_PARTICLES);
    double step = 1.0 / NUM_PARTICLES;
    double r = uniformDist(rng) * step;
    double cumulative = particles[0].weight;
    int idx = 0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        double target = r + i * step;
        while (cumulative < target && idx < (int)particles.size() - 1) {
            idx++;
            cumulative += particles[idx].weight;
        }
        newParticles.push_back({
            particles[idx].x     + gaussianNoise(0, 0.1),
            particles[idx].y     + gaussianNoise(0, 0.1),
            wrapAngle(particles[idx].theta + gaussianNoise(0, 0.005)),
            1.0 / NUM_PARTICLES
        });
    }
    particles = newParticles;
}

Pose mclEstimate() {
    double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
    for (auto& p : particles) {
        sumX   += p.x * p.weight;
        sumY   += p.y * p.weight;
        sumSin += sin(p.theta) * p.weight;
        sumCos += cos(p.theta) * p.weight;
    }
    return { sumX, sumY, atan2(sumSin, sumCos) };
}

// ============================================================
// LOCAL-FRAME ODOMETRY DELTA FROM LEMLIB
// ============================================================
// LemLib's getPose() gives a field-frame pose, but each particle has its
// own heading — so we convert the field-frame delta into the robot's own
// local (forward/strafe) motion using the midpoint heading, then re-apply
// that local motion in each particle's frame in mclPredict() above.
struct LocalDelta { double forward, strafe, dtheta; };

LocalDelta computeLocalDelta(const Pose& last, const Pose& cur) {
    double dx = cur.x - last.x;
    double dy = cur.y - last.y;
    double dtheta = wrapAngle(cur.theta - last.theta);
    double mid = last.theta + dtheta / 2.0;

    double forward =  dx * cos(mid) + dy * sin(mid);
    double strafe  = -dx * sin(mid) + dy * cos(mid);
    return { forward, strafe, dtheta };
}

// ============================================================
// MAIN MCL TASK
// ============================================================
extern lemlib::Chassis chassis;
extern pros::Distance distFront, distBack, distLeft, distRight;

void mclTask() {
    // Wait for chassis.calibrate() to finish in initialize() before this runs.
    Pose lastPose = getLemlibPoseStandard(chassis);
    initMCL(lastPose.x, lastPose.y, lastPose.theta);

    uint32_t lastSensorReadTime = pros::millis();
    const uint32_t SENSOR_PERIOD_MS = 33;  // matches V5 Distance Sensor update rate
    const bool SYNC_MCL_TO_LEMLIB = false;  // flip to true after verifying Section 8

    while (true) {
        // 1. Get current LemLib pose (odom + IMU already fused by LemLib)
        Pose curPose = getLemlibPoseStandard(chassis);

        // 2. Compute this tick's local-frame motion delta
        LocalDelta delta = computeLocalDelta(lastPose, curPose);
        lastPose = curPose;

        // 3. Predict: apply the motion to every particle
        mclPredict(delta.forward, delta.strafe, delta.dtheta);

        // 4. Correct: only when it's time for a new sensor reading (~30 Hz)
        bool correctedThisCycle = false;
        if (pros::millis() - lastSensorReadTime >= SENSOR_PERIOD_MS) {
            double sensorReadings[4] = {
                readValidatedDistance(distFront),
                readValidatedDistance(distBack),
                readValidatedDistance(distLeft),
                readValidatedDistance(distRight)
            };
            // mclWeightAndResample skips itself if no reading is valid this cycle
            mclWeightAndResample(sensorReadings);
            correctedThisCycle = true;  // even a skipped update means we tried
            lastSensorReadTime = pros::millis();
        }

        // 5. Estimate
        Pose mclPose = mclEstimate();

        // 6. OPTIONAL: sync the corrected estimate back into LemLib so that
        //    chassis movements (moveToPose, turnToHeading, etc.) use it too.
        //    Because setPose() has no standardPos argument, its heading
        //    convention may not match getPose(true, true) — verify the
        //    conversion on your LemLib version (see Section 8) before
        //    flipping SYNC_MCL_TO_LEMLIB on. Only sync after a real ToF
        //    correction cycle, not on prediction-only ticks.
        if (SYNC_MCL_TO_LEMLIB && correctedThisCycle) {
            double nativeTheta = PI / 2.0 - mclPose.theta;
            chassis.setPose(mclPose.x, mclPose.y, nativeTheta, true);
            lastPose = getLemlibPoseStandard(chassis);  // re-sync baseline after the jump
        }

        // Use mclPose.x / mclPose.y / mclPose.theta for display (Section 10)
        // and/or decision logic here.

        pros::delay(10);  // ~100 Hz loop
    }
}