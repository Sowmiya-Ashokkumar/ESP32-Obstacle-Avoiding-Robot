#pragma once

/*
 * Generic PID Controller
 * Features: anti-windup (integral clamping), derivative low-pass filter,
 *            output saturation, and runtime gain tuning.
 */
class PIDController {
public:
    // Gains
    float Kp, Ki, Kd;

    // Output limits
    float outMin, outMax;

    // Derivative filter coefficient (0 = no filter, 1 = heavy filter)
    float derivativeFilter;

    PIDController(float kp, float ki, float kd,
                  float minOut = -255.0f, float maxOut = 255.0f,
                  float dFilter = 0.7f)
        : Kp(kp), Ki(ki), Kd(kd),
          outMin(minOut), outMax(maxOut),
          derivativeFilter(dFilter),
          _integral(0), _prevError(0), _prevDerivative(0),
          _initialized(false) {}

    void reset() {
        _integral      = 0;
        _prevError     = 0;
        _prevDerivative = 0;
        _initialized   = false;
    }

    /*
     * Compute PID output.
     * @param setpoint  - desired value
     * @param measured  - current measured value
     * @param dt        - time delta in seconds
     * @return          - control output (clamped to [outMin, outMax])
     */
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measured;

        // --- Proportional ---
        float P = Kp * error;

        // --- Integral with anti-windup (clamping) ---
        _integral += error * dt;
        float rawOut_noD = P + Ki * _integral;
        if (rawOut_noD > outMax)      _integral = (outMax - P) / Ki;
        else if (rawOut_noD < outMin) _integral = (outMin - P) / Ki;

        float I = Ki * _integral;

        // --- Derivative with low-pass filter ---
        float derivative = 0.0f;
        if (_initialized) {
            float rawD = (error - _prevError) / dt;
            // Low-pass: D_filtered = α * D_prev + (1-α) * D_raw
            derivative = derivativeFilter * _prevDerivative
                         + (1.0f - derivativeFilter) * rawD;
        }
        _prevDerivative = derivative;
        _prevError      = error;
        _initialized    = true;

        float D = Kd * derivative;

        // --- Sum and clamp ---
        float output = P + I + D;
        if      (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;

        return output;
    }

    // Convenience getters for telemetry
    float getIntegral()    const { return _integral; }
    float getPrevError()   const { return _prevError; }

private:
    float _integral;
    float _prevError;
    float _prevDerivative;
    bool  _initialized;
};
