#ifndef RCJTAIWAN_H
#define RCJTAIWAN_H

#include <sub_core.h>

// ============================================================
// RCJTaiwan.h — Basic Robot Control Library
// Depends on sub_core.h (FC_Vector_Motion, RobotIKControl,
//   update_gyro_sensor, fast_update_line_sensor, lineData,
//   gyroData, robot, RobotPos, read_cam_and_pos_data)
// ============================================================


// ------------------------------------------------------------
// LINE SENSOR MASKS
// front:  bits 5-11
// right:  bits 12-20
// back:   bits 21-27
// left:   bits 28-31 + 0-4
// ------------------------------------------------------------
#define LS_MASK_FRONT  0x00000FE0UL
#define LS_MASK_RIGHT  0x001FF000UL
#define LS_MASK_BACK   0x0FE00000UL
#define LS_MASK_LEFT   0xF000001FUL

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f


// ------------------------------------------------------------
// MOVEMENT
// ------------------------------------------------------------

// Move at world-frame velocity, non-blocking (call repeatedly in a loop)
inline void moveRobot(float vx, float vy, float heading = 90.0f) {
    FC_Vector_Motion(vx, vy, heading);
}

// Stop all motors
inline void stopRobot() {
    FC_Vector_Motion(0, 0, 90);
}

// Turn to an absolute heading (degrees). Blocks until done or timeout.
inline bool turnToHeading(float targetHeading, float threshold = 5.0f,
                          uint32_t timeout_ms = 3000) {
    uint32_t start = millis();
    while (1) {
        update_gyro_sensor();
        float current = 90.0f - gyroData.heading;
        float e = targetHeading - current;
        while (e >  180) e -= 360;
        while (e < -180) e += 360;
        if (fabs(e) <= threshold) { stopRobot(); return true; }
        if (timeout_ms && millis() - start > timeout_ms) { stopRobot(); return false; }
        float omega = constrain(e * robot.P_factor, -60.0f, 60.0f);
        RobotIKControl(0, 0, omega);
    }
}


// ------------------------------------------------------------
// TIMED ACTIONS
// ------------------------------------------------------------

// Move for a fixed duration (ms), then stop
inline void moveForMs(float vx, float vy, uint32_t duration_ms,
                      float heading = 90.0f) {
    uint32_t start = millis();
    while (millis() - start < duration_ms) {
        update_gyro_sensor();
        FC_Vector_Motion(vx, vy, heading);
    }
    stopRobot();
}

// Rotate in place for a fixed duration (ms), then stop
inline void rotateForMs(float omega, uint32_t duration_ms) {
    uint32_t start = millis();
    while (millis() - start < duration_ms) {
        update_gyro_sensor();
        RobotIKControl(0, 0, omega);
    }
    stopRobot();
}


// ------------------------------------------------------------
// CONDITION-BASED — LINE SENSOR
// ------------------------------------------------------------

// Returns true if ANY sensor in mask is triggered (bit == 0)
inline bool isLineTouched(uint32_t mask) {
    return (lineData.state & mask) != mask;
}

// Move until the specified line sensor mask triggers, then stop.
// Returns true on touch, false on timeout.
inline bool moveUntilTouch(float vx, float vy, uint32_t mask,
                           uint32_t timeout_ms = 5000, float heading = 90.0f) {
    uint32_t start = millis();
    while (1) {
        update_gyro_sensor();
        fast_update_line_sensor();
        FC_Vector_Motion(vx, vy, heading);
        if (isLineTouched(mask))                          { stopRobot(); return true;  }
        if (timeout_ms && millis() - start > timeout_ms)  { stopRobot(); return false; }
    }
}

// Shorthand wrappers — pass the speed relevant to each direction
// e.g. moveUntilFrontTouch(0, 40) drives forward until front line
inline bool moveUntilFrontTouch(float vx, float vy, uint32_t timeout_ms = 5000, float heading = 90.0f) {
    return moveUntilTouch(vx, vy, LS_MASK_FRONT, timeout_ms, heading);
}
inline bool moveUntilBackTouch(float vx, float vy, uint32_t timeout_ms = 5000, float heading = 90.0f) {
    return moveUntilTouch(vx, vy, LS_MASK_BACK, timeout_ms, heading);
}
inline bool moveUntilLeftTouch(float vx, float vy, uint32_t timeout_ms = 5000, float heading = 90.0f) {
    return moveUntilTouch(vx, vy, LS_MASK_LEFT, timeout_ms, heading);
}
inline bool moveUntilRightTouch(float vx, float vy, uint32_t timeout_ms = 5000, float heading = 90.0f) {
    return moveUntilTouch(vx, vy, LS_MASK_RIGHT, timeout_ms, heading);
}


// ------------------------------------------------------------
// CONDITION-BASED — POSITION
// ------------------------------------------------------------

// Move until within tolerance of (target_x, target_y).
// Uses RobotPos updated by read_cam_and_pos_data().
// Returns true on success, false on timeout.
inline bool moveUntilPosition(int target_x, int target_y,
                              int tolerance = 10, uint32_t timeout_ms = 5000) {
    uint32_t start = millis();
    while (1) {
        update_gyro_sensor();
        read_cam_and_pos_data();

        int dx = target_x - RobotPos.x;
        int dy = target_y - RobotPos.y;

        if (abs(dx) <= tolerance && abs(dy) <= tolerance) { stopRobot(); return true; }
        if (timeout_ms && millis() - start > timeout_ms)  { stopRobot(); return false; }

        float vx = constrain(dx * 0.3f, -40.0f, 40.0f);
        float vy = constrain(dy * 0.3f, -40.0f, 40.0f);
        if (vx != 0 && fabs(vx) < 15) vx = (vx > 0) ? 15.0f : -15.0f;
        if (vy != 0 && fabs(vy) < 15) vy = (vy > 0) ? 15.0f : -15.0f;

        FC_Vector_Motion(vx, vy, 90);
    }
}

#endif // RCJTAIWAN_H
