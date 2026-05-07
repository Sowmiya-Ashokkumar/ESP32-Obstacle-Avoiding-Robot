#pragma once
#include <math.h>

/*
 * Complementary Filter for IMU Angle Estimation
 *
 * Fuses:
 *   - Gyroscope  : accurate short-term (integrate angular rate)
 *   - Accelerometer : accurate long-term (gravity reference)
 *
 * Formula:
 *   angle = α * (angle + gyro_rate * dt) + (1 - α) * accel_angle
 *
 * α close to 1 → trust gyro more (less noise, but drifts over time)
 * α close to 0 → trust accelerometer more (no drift, but noisy)
 * Typical: α = 0.96 – 0.98
 */
class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = 0.96f)
        : _alpha(alpha), _angleX(0.0f), _angleY(0.0f) {}

    /*
     * Update filter with raw IMU readings.
     * @param ax, ay, az  - raw accelerometer (any consistent unit, e.g. m/s²)
     * @param gx, gy      - gyroscope in degrees/second
     * @param dt          - time delta in seconds
     */
    void update(float ax, float ay, float az,
                float gx, float gy, float dt) {

        // Accelerometer angle (degrees) — using atan2 for full quadrant coverage
        float accelAngleX = atan2f(ay, az) * RAD_TO_DEG;
        float accelAngleY = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

        // Integrate gyroscope
        float gyroAngleX = _angleX + gx * dt;
        float gyroAngleY = _angleY + gy * dt;

        // Complementary fusion
        _angleX = _alpha * gyroAngleX + (1.0f - _alpha) * accelAngleX;
        _angleY = _alpha * gyroAngleY + (1.0f - _alpha) * accelAngleY;
    }

    float getAngleX() const { return _angleX; }  // Roll
    float getAngleY() const { return _angleY; }  // Pitch

    void setAlpha(float alpha) { _alpha = alpha; }
    void reset(float angleX = 0.0f, float angleY = 0.0f) {
        _angleX = angleX;
        _angleY = angleY;
    }

private:
    float _alpha;
    float _angleX;
    float _angleY;
};
