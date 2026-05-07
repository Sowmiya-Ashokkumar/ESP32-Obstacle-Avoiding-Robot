#pragma once

/*
 * Autonomous Robot Finite State Machine
 *
 * States:
 *   IDLE             → waiting for start command
 *   MOVING_FORWARD   → driving straight, PID maintaining heading
 *   OBSTACLE_AHEAD   → obstacle within warning distance, slowing down
 *   AVOIDING_LEFT    → turning left to avoid obstacle
 *   AVOIDING_RIGHT   → turning right (fallback if left blocked too)
 *   EMERGENCY_STOP   → obstacle too close, full stop
 *   FAULT            → tilt/fall detected (angle too large)
 *
 * Transitions driven by:
 *   - Ultrasonic distance
 *   - IMU tilt angle
 *   - Elapsed time in state
 */

enum class RobotState : uint8_t {
    IDLE = 0,
    MOVING_FORWARD,
    OBSTACLE_AHEAD,
    AVOIDING_LEFT,
    AVOIDING_RIGHT,
    EMERGENCY_STOP,
    FAULT
};

// Threshold constants
#define DIST_WARNING_CM     40.0f   // Start slowing down
#define DIST_STOP_CM        15.0f   // Full stop
#define TILT_FAULT_DEG      45.0f   // Robot fallen over
#define AVOID_DURATION_MS   1500    // Time to turn before re-checking
#define START_BUTTON_PIN    0       // GPIO0 = BOOT button on most ESP32 boards

struct RobotStatus {
    RobotState state;
    float      distanceCm;
    float      rollDeg;
    float      pitchDeg;
    float      motorLeft;   // -255 to 255 (PWM duty)
    float      motorRight;
    uint32_t   stateAge;    // ms since last state change
};

class RobotFSM {
public:
    RobotFSM() : _state(RobotState::IDLE), _stateStart(0), _avoidDir(1) {}

    void begin() {
        pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    }

    /*
     * Call this from the control task on every cycle.
     * Reads sensors, runs state machine, writes motor commands.
     */
    void update(RobotStatus& status) {
        uint32_t now = millis();
        status.stateAge = now - _stateStart;

        // --- Global fault check (overrides everything) ---
        if (fabsf(status.rollDeg) > TILT_FAULT_DEG ||
            fabsf(status.pitchDeg) > TILT_FAULT_DEG) {
            _transition(RobotState::FAULT, status);
        }

        switch (_state) {

            case RobotState::IDLE:
                status.motorLeft  = 0;
                status.motorRight = 0;
                // Start when button pressed (active LOW)
                if (digitalRead(START_BUTTON_PIN) == LOW) {
                    _transition(RobotState::MOVING_FORWARD, status);
                }
                break;

            case RobotState::MOVING_FORWARD:
                if (status.distanceCm < DIST_STOP_CM) {
                    _transition(RobotState::EMERGENCY_STOP, status);
                } else if (status.distanceCm < DIST_WARNING_CM) {
                    _transition(RobotState::OBSTACLE_AHEAD, status);
                }
                // Motor commands set by PID in main loop
                break;

            case RobotState::OBSTACLE_AHEAD:
                // Slow down proportionally
                {
                    float factor = (status.distanceCm - DIST_STOP_CM)
                                   / (DIST_WARNING_CM - DIST_STOP_CM);
                    status.motorLeft  *= factor;
                    status.motorRight *= factor;
                }
                if (status.distanceCm < DIST_STOP_CM) {
                    _transition(RobotState::EMERGENCY_STOP, status);
                } else if (status.distanceCm >= DIST_WARNING_CM) {
                    _transition(RobotState::MOVING_FORWARD, status);
                } else if (status.stateAge > 2000) {
                    // Still blocked after 2 s → start avoidance
                    _avoidDir = 1;
                    _transition(RobotState::AVOIDING_LEFT, status);
                }
                break;

            case RobotState::AVOIDING_LEFT:
                // Spin left
                status.motorLeft  = -150;
                status.motorRight =  150;
                if (status.stateAge > AVOID_DURATION_MS) {
                    if (status.distanceCm > DIST_WARNING_CM) {
                        _transition(RobotState::MOVING_FORWARD, status);
                    } else {
                        // Try right instead
                        _transition(RobotState::AVOIDING_RIGHT, status);
                    }
                }
                break;

            case RobotState::AVOIDING_RIGHT:
                // Spin right
                status.motorLeft  =  150;
                status.motorRight = -150;
                if (status.stateAge > AVOID_DURATION_MS) {
                    // Go forward regardless; re-evaluate next cycle
                    _transition(RobotState::MOVING_FORWARD, status);
                }
                break;

            case RobotState::EMERGENCY_STOP:
                status.motorLeft  = 0;
                status.motorRight = 0;
                // Resume when clear
                if (status.distanceCm >= DIST_WARNING_CM && status.stateAge > 500) {
                    _transition(RobotState::MOVING_FORWARD, status);
                }
                break;

            case RobotState::FAULT:
                status.motorLeft  = 0;
                status.motorRight = 0;
                // Only recover manually (button press)
                if (fabsf(status.rollDeg) < 20.0f &&
                    fabsf(status.pitchDeg) < 20.0f &&
                    digitalRead(START_BUTTON_PIN) == LOW) {
                    _transition(RobotState::IDLE, status);
                }
                break;
        }

        status.state = _state;
    }

    RobotState getState() const { return _state; }

    static const char* stateName(RobotState s) {
        switch (s) {
            case RobotState::IDLE:             return "IDLE";
            case RobotState::MOVING_FORWARD:   return "MOVING_FWD";
            case RobotState::OBSTACLE_AHEAD:   return "OBSTACLE";
            case RobotState::AVOIDING_LEFT:    return "AVOID_LEFT";
            case RobotState::AVOIDING_RIGHT:   return "AVOID_RIGHT";
            case RobotState::EMERGENCY_STOP:   return "E_STOP";
            case RobotState::FAULT:            return "FAULT";
            default:                           return "UNKNOWN";
        }
    }

private:
    RobotState _state;
    uint32_t   _stateStart;
    int8_t     _avoidDir;

    void _transition(RobotState next, RobotStatus& status) {
        if (_state == next) return;
        Serial.printf("[FSM] %s → %s\n",
                      stateName(_state), stateName(next));
        _state      = next;
        _stateStart = millis();
    }
};
