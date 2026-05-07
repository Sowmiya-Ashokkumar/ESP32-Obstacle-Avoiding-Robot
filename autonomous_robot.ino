/*
 * ============================================================
 *  Autonomous Robot — ESP32 + FreeRTOS
 * ============================================================
 *  Hardware (simulate on https://wokwi.com):
 *    - ESP32 DevKit
 *    - MPU6050  (I2C: SDA=21, SCL=22)
 *    - HC-SR04  (TRIG=5, ECHO=18)
 *    - Motor A  (PWM=25, DIR=26)  → left wheel
 *    - Motor B  (PWM=32, DIR=33)  → right wheel
 *    - Start button on GPIO0 (built-in BOOT button)
 *
 *  Architecture — three FreeRTOS tasks:
 *    1. SensorTask   (100 Hz, Core 1) — read IMU + ultrasonic
 *    2. ControlTask  ( 50 Hz, Core 1) — PID + FSM → motor PWM
 *    3. TelemetryTask( 10 Hz, Core 0) — serial print dashboard
 *
 *  Shared data protected by a FreeRTOS mutex.
 * ============================================================
 */

#include <Wire.h>
#include "PIDController.h"
#include "ComplementaryFilter.h"
#include "RobotFSM.h"

// ── Pin definitions ────────────────────────────────────────
#define PIN_SDA       21
#define PIN_SCL       22
#define PIN_TRIG       5
#define PIN_ECHO      18
#define PIN_MOTOR_A_PWM  25
#define PIN_MOTOR_A_DIR  26
#define PIN_MOTOR_B_PWM  32
#define PIN_MOTOR_B_DIR  33

// ── MPU6050 I2C ────────────────────────────────────────────
#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT   0x6B
#define MPU6050_ACCEL_XOUT 0x3B
#define MPU6050_GYRO_XOUT  0x43
#define ACCEL_SCALE        16384.0f   // ±2g  → LSB/g
#define GYRO_SCALE         131.0f     // ±250 °/s → LSB/(°/s)

// ── PWM channels (ESP32 LEDC) ──────────────────────────────
#define PWM_FREQ    1000   // 1 kHz
#define PWM_RES     8      // 8-bit (0–255)
#define CH_MOTOR_A  0
#define CH_MOTOR_B  1

// ── Shared state (protected by mutex) ─────────────────────
static SemaphoreHandle_t gMutex;
static RobotStatus       gStatus;

// ── Module instances ───────────────────────────────────────
static PIDController       headingPID(3.5f, 0.1f, 0.8f, -200.0f, 200.0f);
static PIDController       speedPID(4.0f, 0.2f, 0.5f, -255.0f, 255.0f);
static ComplementaryFilter imuFilter(0.97f);
static RobotFSM            fsm;

// ── Gyro bias (calibrated at startup) ─────────────────────
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ── Forward declarations ───────────────────────────────────
void   mpu6050Init();
void   mpu6050Read(float& ax, float& ay, float& az,
                   float& gx, float& gy, float& gz);
float  measureDistance();
void   driveMotors(float left, float right);
void   calibrateGyro(int samples = 200);
void   sensorTask(void* pv);
void   controlTask(void* pv);
void   telemetryTask(void* pv);

// ===========================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Autonomous Robot Booting ===");

    // I2C + MPU6050
    Wire.begin(PIN_SDA, PIN_SCL);
    mpu6050Init();
    calibrateGyro(300);

    // Ultrasonic
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    // Motors via LEDC
    ledcSetup(CH_MOTOR_A, PWM_FREQ, PWM_RES);
    ledcSetup(CH_MOTOR_B, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_MOTOR_A_PWM, CH_MOTOR_A);
    ledcAttachPin(PIN_MOTOR_B_PWM, CH_MOTOR_B);
    pinMode(PIN_MOTOR_A_DIR, OUTPUT);
    pinMode(PIN_MOTOR_B_DIR, OUTPUT);
    driveMotors(0, 0);

    // FSM start button
    fsm.begin();

    // Shared state
    memset(&gStatus, 0, sizeof(gStatus));
    gStatus.state = RobotState::IDLE;
    gMutex = xSemaphoreCreateMutex();

    // ── Launch FreeRTOS tasks ──────────────────────────────
    // SensorTask: highest priority, Core 1
    xTaskCreatePinnedToCore(sensorTask,    "Sensor",    4096, NULL, 3, NULL, 1);
    // ControlTask: medium priority, Core 1
    xTaskCreatePinnedToCore(controlTask,   "Control",   4096, NULL, 2, NULL, 1);
    // TelemetryTask: low priority, Core 0
    xTaskCreatePinnedToCore(telemetryTask, "Telemetry", 4096, NULL, 1, NULL, 0);

    Serial.println("[BOOT] All tasks started. Press BOOT button to begin.");
}

void loop() {
    // Empty — all work done in FreeRTOS tasks
    vTaskDelay(portMAX_DELAY);
}

// ===========================================================
//  TASK 1 — Sensor reading @ 100 Hz
// ===========================================================
void sensorTask(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(10);   // 10 ms = 100 Hz
    TickType_t       lastWake = xTaskGetTickCount();

    float ax, ay, az, gx, gy, gz;
    uint32_t prevTime = millis();

    while (true) {
        uint32_t now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;
        if (dt <= 0 || dt > 0.1f) dt = 0.01f;   // Guard

        // Read IMU
        mpu6050Read(ax, ay, az, gx, gy, gz);

        // Apply gyro bias correction
        gx -= gyroBiasX;
        gy -= gyroBiasY;
        gz -= gyroBiasZ;

        // Fuse with complementary filter
        imuFilter.update(ax, ay, az, gx, gy, gz, dt);

        // Ultrasonic distance
        float dist = measureDistance();

        // Write to shared state
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gStatus.rollDeg    = imuFilter.getAngleX();
            gStatus.pitchDeg   = imuFilter.getAngleY();
            gStatus.distanceCm = dist;
            xSemaphoreGive(gMutex);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ===========================================================
//  TASK 2 — Control loop @ 50 Hz  (PID + FSM)
// ===========================================================
void controlTask(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(20);  // 20 ms = 50 Hz
    TickType_t       lastWake = xTaskGetTickCount();

    float targetHeading = 0.0f;     // Drive straight (0° yaw correction)
    float targetSpeed   = 180.0f;   // Base PWM when moving

    while (true) {
        RobotStatus local;

        // Snapshot shared state
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local = gStatus;
            xSemaphoreGive(gMutex);
        }

        // Compute heading correction using roll as proxy for drift
        float headingCorrection = headingPID.compute(targetHeading,
                                                     local.rollDeg,
                                                     0.02f);

        // Compute speed (with ramp when close to obstacle)
        float baseSpeed = speedPID.compute(targetSpeed,
                                           targetSpeed,  // open-loop base
                                           0.02f);
        baseSpeed = targetSpeed;  // Open-loop base; FSM will scale it

        // Differential drive: correction steers the robot
        local.motorLeft  = baseSpeed + headingCorrection;
        local.motorRight = baseSpeed - headingCorrection;

        // Clamp to valid PWM range
        local.motorLeft  = constrain(local.motorLeft,  -255.0f, 255.0f);
        local.motorRight = constrain(local.motorRight, -255.0f, 255.0f);

        // Run FSM (may override motor values)
        fsm.update(local);

        // Apply motors
        driveMotors(local.motorLeft, local.motorRight);

        // Write back updated status
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            gStatus.motorLeft  = local.motorLeft;
            gStatus.motorRight = local.motorRight;
            gStatus.state      = local.state;
            gStatus.stateAge   = local.stateAge;
            xSemaphoreGive(gMutex);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

// ===========================================================
//  TASK 3 — Serial telemetry dashboard @ 10 Hz
// ===========================================================
void telemetryTask(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(100);  // 100 ms = 10 Hz
    TickType_t       lastWake = xTaskGetTickCount();

    while (true) {
        RobotStatus s;
        if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s = gStatus;
            xSemaphoreGive(gMutex);
        }

        // ASCII dashboard
        Serial.println("┌─────────────────────────────────────────┐");
        Serial.printf( "│ State    : %-28s  │\n", RobotFSM::stateName(s.state));
        Serial.printf( "│ Distance : %6.1f cm                     │\n", s.distanceCm);
        Serial.printf( "│ Roll     : %+7.2f °  Pitch: %+7.2f °   │\n",
                       s.rollDeg, s.pitchDeg);
        Serial.printf( "│ Motor L  : %+7.1f    Motor R: %+7.1f   │\n",
                       s.motorLeft, s.motorRight);
        Serial.printf( "│ State age: %lu ms                        │\n",
                       s.stateAge);
        Serial.println("└─────────────────────────────────────────┘");

        vTaskDelayUntil(&lastWake, period);
    }
}

// ===========================================================
//  MPU6050 helpers
// ===========================================================
void mpu6050Init() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT);
    Wire.write(0x00);   // Wake up
    Wire.endTransmission();
    delay(100);
    Serial.println("[IMU] MPU6050 initialized.");
}

void mpu6050Read(float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    auto read16 = []() -> int16_t {
        return (int16_t)((Wire.read() << 8) | Wire.read());
    };

    ax = read16() / ACCEL_SCALE;
    ay = read16() / ACCEL_SCALE;
    az = read16() / ACCEL_SCALE;
    /* skip temperature */ read16();
    gx = read16() / GYRO_SCALE;
    gy = read16() / GYRO_SCALE;
    gz = read16() / GYRO_SCALE;
}

void calibrateGyro(int samples) {
    Serial.println("[IMU] Calibrating gyro — keep robot still...");
    double bx = 0, by = 0, bz = 0;
    float ax, ay, az, gx, gy, gz;
    for (int i = 0; i < samples; i++) {
        mpu6050Read(ax, ay, az, gx, gy, gz);
        bx += gx; by += gy; bz += gz;
        delay(5);
    }
    gyroBiasX = bx / samples;
    gyroBiasY = by / samples;
    gyroBiasZ = bz / samples;
    Serial.printf("[IMU] Gyro bias: X=%.4f  Y=%.4f  Z=%.4f\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);
}

// ===========================================================
//  HC-SR04 ultrasonic
// ===========================================================
float measureDistance() {
    // Trigger 10 µs pulse
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Measure echo (timeout = 30 ms → ~500 cm max)
    long duration = pulseIn(PIN_ECHO, HIGH, 30000);
    if (duration == 0) return 400.0f;   // No echo → far away
    return (duration * 0.0343f) / 2.0f;
}

// ===========================================================
//  Motor driver  (L298N / DRV8833 style)
// ===========================================================
void driveMotors(float left, float right) {
    // Motor A (left)
    digitalWrite(PIN_MOTOR_A_DIR, left >= 0 ? HIGH : LOW);
    ledcWrite(CH_MOTOR_A, (uint32_t)fabsf(left));

    // Motor B (right)
    digitalWrite(PIN_MOTOR_B_DIR, right >= 0 ? HIGH : LOW);
    ledcWrite(CH_MOTOR_B, (uint32_t)fabsf(right));
}
