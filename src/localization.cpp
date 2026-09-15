#include "localization.hpp" // IWYU pragma: keep


// ============================================================
// POSE STRUCT
// ============================================================
struct Pose {
    double x;
    double y;
    double theta;  // radians
};

// ============================================================
// ODOMETRY CLASS
// ============================================================
class Odometry {
private:
    double x, y, theta;
    double lastFwdPos, lastSidePos, lastHeading;
    pros::Imu& imu;
    pros::Rotation& fwdWheel;
    pros::Rotation& sideWheel;

public:
    Odometry(pros::Imu& imu_, pros::Rotation& fwd_, pros::Rotation& side_)
        : imu(imu_), fwdWheel(fwd_), sideWheel(side_) {
        x = y = theta = 0;
        lastFwdPos = lastSidePos = lastHeading = 0;
    }

    void setPose(double nx, double ny, double ntheta) {
        x = nx; y = ny; theta = ntheta;
    }

    Pose getPose() { return {x, y, theta}; }

    void update() {
        // Read sensors
        double fwdPos = fwdWheel.get_position();    // centidegrees
        double sidePos = sideWheel.get_position();
        double headingDeg = imu.get_heading();
        double headingRad = headingDeg * M_PI / 180.0;

        // Compute deltas
        double fwdDelta = (fwdPos - lastFwdPos) / 100.0 * DEG_TO_INCH;  // /100 for centidegrees
        double sideDelta = (sidePos - lastSidePos) / 100.0 * DEG_TO_INCH;

        // Handle heading wraparound
        double deltaTheta = headingRad - lastHeading;
        if (deltaTheta > M_PI)  deltaTheta -= 2 * M_PI;
        if (deltaTheta < -M_PI) deltaTheta += 2 * M_PI;

        // Subtract rotational component from tracking wheel readings
        double deltaF = fwdDelta - FWD_WHEEL_OFFSET * deltaTheta;
        double deltaS = sideDelta - SIDE_WHEEL_OFFSET * deltaTheta;

        // Rotate to field frame using midpoint heading
        double thetaMid = theta + deltaTheta / 2.0;

        if (fabs(deltaTheta) < 0.0001) {
            // Straight line — simple case
            x += deltaF * cos(theta) - deltaS * sin(theta);
            y += deltaF * sin(theta) + deltaS * cos(theta);
        } else {
            // Arc-based correction (Pilons method)
            double sinHalfDelta = sin(deltaTheta / 2.0);
            double chordLength = 2.0 * sinHalfDelta;
            x += chordLength * (deltaF / deltaTheta) * cos(thetaMid)
               - chordLength * (deltaS / deltaTheta) * sin(thetaMid);
            y += chordLength * (deltaF / deltaTheta) * sin(thetaMid)
               + chordLength * (deltaS / deltaTheta) * cos(thetaMid);
        }

        theta = headingRad;

        // Store for next iteration
        lastFwdPos = fwdPos;
        lastSidePos = sidePos;
        lastHeading = headingRad;
    }
};

// ============================================================
// DISTANCE SENSOR HELPER
// ============================================================
double getFilteredDistance(pros::Distance& sensor) {
    double sum = 0;
    int validCount = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        double reading = sensor.get() / 25.4;  // mm to inches
        if (reading >= 20.0 / 25.4 && reading <= 2000.0 / 25.4) {
            sum += reading;
            validCount++;
        }
        pros::delay(5);
    }
    if (validCount == 0) return -1.0;  // no valid readings
    return sum / validCount;
}

// ============================================================
// RAYCAST — Expected distance from sensor to nearest field wall
// ============================================================
double raycastToWall(double x, double y, double theta, const SensorConfig& sensor) {
    // Sensor position in field frame
    double cosT = cos(theta);
    double sinT = sin(theta);
    double psx = x + (sensor.sx * cosT - sensor.sy * sinT);
    double psy = y + (sensor.sx * sinT + sensor.sy * cosT);

    // Beam direction in field frame
    double phi = theta + sensor.mountingAngle;
    double dx = cos(phi);
    double dy = sin(phi);

    // Find nearest wall intersection
    double minDist = 1e9;

    // Wall X=0 (left wall)
    if (dx < -0.001) {
        double t = -psx / dx;
        if (t > 0 && t < minDist) minDist = t;
    }
    // Wall X=FIELD_WIDTH (right wall)
    if (dx > 0.001) {
        double t = (FIELD_WIDTH - psx) / dx;
        if (t > 0 && t < minDist) minDist = t;
    }
    // Wall Y=0 (back wall)
    if (dy < -0.001) {
        double t = -psy / dy;
        if (t > 0 && t < minDist) minDist = t;
    }
    // Wall Y=FIELD_HEIGHT (front wall)
    if (dy > 0.001) {
        double t = (FIELD_HEIGHT - psy) / dy;
        if (t > 0 && t < minDist) minDist = t;
    }

    return minDist;
}

// ============================================================
// SIMPLE FUSION (Blend Correction)
// ============================================================
Pose simpleFusion(Odometry& odom, pros::Distance sensors[4], Pose currentPose) {
    double totalXCorrection = 0;
    double totalYCorrection = 0;
    int validSensors = 0;

    for (int i = 0; i < 4; i++) {
        double reading = getFilteredDistance(sensors[i]);
        if (reading < 0) continue;  // no valid reading

        double expected = raycastToWall(currentPose.x, currentPose.y,
                                        currentPose.theta, SENSORS[i]);

        double residual = reading - expected;
        if (fabs(residual) > MAX_RESIDUAL) continue;  // reject outlier

        // Project correction onto field axes
        double beamAngle = currentPose.theta + SENSORS[i].mountingAngle;
        totalXCorrection += residual * cos(beamAngle);
        totalYCorrection += residual * sin(beamAngle);
        validSensors++;
    }

    if (validSensors > 0) {
        double avgX = totalXCorrection / validSensors;
        double avgY = totalYCorrection / validSensors;
        currentPose.x += BLEND_WEIGHT * avgX;
        currentPose.y += BLEND_WEIGHT * avgY;
    }

    odom.setPose(currentPose.x, currentPose.y, currentPose.theta);
    return currentPose;
}

// ============================================================
// MCL (Particle Filter)
// ============================================================
std::mt19937 rng(42);
std::normal_distribution<double> noiseDist(0.0, 1.0);

double gaussianNoise(double mean, double stddev) {
    return mean + noiseDist(rng) * stddev;
}

struct Particle {
    double x, y, theta, weight;
};

std::vector<Particle> particles;

void initMCL(double startX, double startY, double startTheta) {
    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({
            startX + gaussianNoise(0, 1.0),
            startY + gaussianNoise(0, 1.0),
            startTheta + gaussianNoise(0, 0.05),
            1.0 / NUM_PARTICLES
        });
    }
}

double gaussianProb(double actual, double expected, double sigma) {
    double diff = actual - expected;
    return exp(-0.5 * (diff * diff) / (sigma * sigma));
}

Pose mclUpdate(double odomDX, double odomDY, double odomDTheta,
               pros::Distance sensors[4]) {

    // --- PREDICTION ---
    for (auto& p : particles) {
        p.x += odomDX + gaussianNoise(0, SIGMA_XY);
        p.y += odomDY + gaussianNoise(0, SIGMA_XY);
        p.theta += odomDTheta + gaussianNoise(0, SIGMA_THETA);
    }

    // --- WEIGHTING ---
    double totalWeight = 0;
    for (auto& p : particles) {
        p.weight = 1.0;
        for (int i = 0; i < 4; i++) {
            double reading = sensors[i].get() / 25.4;  // mm to inches
            if (reading < 0.79 || reading > 78.74) continue;

            double expected = raycastToWall(p.x, p.y, p.theta, SENSORS[i]);
            double prob = gaussianProb(reading, expected, SIGMA_SENSOR);
            p.weight *= prob;  // use *= for product, += for sum
        }
        totalWeight += p.weight;
    }

    // Avoid division by zero
    if (totalWeight == 0) {
        for (auto& p : particles) p.weight = 1.0 / NUM_PARTICLES;
    } else {
        for (auto& p : particles) p.weight /= totalWeight;
    }

    // --- RESAMPLING (systematic) ---
    std::vector<Particle> newParticles;
    double step = 1.0 / NUM_PARTICLES;
    double r = ((double)rand() / RAND_MAX) * step;
    double cumulative = 0;
    int idx = 0;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        double target = r + i * step;
        while (idx < NUM_PARTICLES - 1 && cumulative < target) {
            cumulative += particles[idx].weight;
            idx++;
        }
        newParticles.push_back({
            particles[idx].x + gaussianNoise(0, 0.1),
            particles[idx].y + gaussianNoise(0, 0.1),
            particles[idx].theta + gaussianNoise(0, 0.005),
            1.0 / NUM_PARTICLES
        });
    }
    particles = newParticles;

    // --- ESTIMATE ---
    double estX = 0, estY = 0, estTheta = 0;
    for (auto& p : particles) {
        estX += p.x * p.weight;
        estY += p.y * p.weight;
        estTheta += p.theta * p.weight;
    }

    return {estX, estY, estTheta};
}

// ============================================================
// MAIN TASK
// ============================================================
void localizationTask(void* params) {
    // Initialize sensors
    pros::Imu imu(IMU_PORT);
    pros::Rotation fwdWheel(FWD_WHEEL_PORT);
    pros::Rotation sideWheel(SIDE_WHEEL_PORT);
    pros::Distance distSensors[4] = {
        pros::Distance(PORT_FRONT),
        pros::Distance(PORT_LEFT),
        pros::Distance(PORT_BACK),
        pros::Distance(PORT_RIGHT)
    };

    // Calibrate IMU
    imu.reset();
    pros::delay(2000);

    // Initialize odometry
    Odometry odom(imu, fwdWheel, sideWheel);
    odom.setPose(START_X, START_Y, START_THETA);

    // Initialize MCL (if using)
    initMCL(START_X, START_Y, START_THETA);

    Pose lastPose = odom.getPose();

    while (true) {
        // Step 1: Update odometry (fast, every 10ms)
        odom.update();
        Pose currentPose = odom.getPose();

        // Compute odometry deltas for MCL
        double odomDX = currentPose.x - lastPose.x;
        double odomDY = currentPose.y - lastPose.y;
        double odomDTheta = currentPose.theta - lastPose.theta;

        // Step 2: Distance sensor correction (slower, ~30Hz)
        // Only run when we have new ToF data
        // (V5 Distance Sensor updates every ~33ms)

        // OPTION A: Simple blend fusion
        // currentPose = simpleFusion(odom, distSensors, currentPose);

        // OPTION B: MCL
        // Pose mclPose = mclUpdate(odomDX, odomDY, odomDTheta, distSensors);
        // odom.setPose(mclPose.x, mclPose.y, mclPose.theta);
        // currentPose = mclPose;

        lastPose = currentPose;

        // Output for motion control
        // (use currentPose.x, currentPose.y, currentPose.theta)

        pros::delay(10);  // 100 Hz
    }
}