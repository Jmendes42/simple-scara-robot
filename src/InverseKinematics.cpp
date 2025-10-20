
#include "InverseKinematics.h"

#include <Arduino.h>
#include <cmath>


InverseKinematics::JointAngles InverseKinematics::calculateJointAngles(float const x_in, float const y_in)
{
    // --- 1. Swap coordinates if physical frame rotated 90° ---
    float const x = y_in;  // horizontal
    float const y = x_in;  // vertical

    // --- 2. Link lengths ---
    float constexpr L1 = 6.0f;
    float constexpr L2 = 4.7f;

    // --- 3. Squares for reuse ---
    float const xSqr = x * x;
    float const ySqr = y * y;
    float const L1Sqr = L1 * L1;
    float const L2Sqr = L2 * L2;

    // --- 4. IK core ---
    float cos_beta = (L1Sqr + L2Sqr - xSqr - ySqr) / (2 * L1 * L2);
    cos_beta = constrain(cos_beta, -1.0f, 1.0f);
    float const beta = acos(cos_beta);   // elbow angle

    float cos_alpha = (xSqr + ySqr + L1Sqr - L2Sqr) / (2 * L1 * sqrt(xSqr + ySqr));
    cos_alpha = constrain(cos_alpha, -1.0f, 1.0f);
    float const alpha = acos(cos_alpha);

    float const phi = atan2(y, x);        // angle from base to target

    // --- 5. Compute joint angles ---
    float const theta1_rad = phi - alpha;        // shoulder
    float const theta2_rad = M_PI - beta;        // elbow

    // --- 6. Convert to degrees ---
    float theta1_deg = theta1_rad * 180.0f / M_PI;
    float theta2_deg = theta2_rad * 180.0f / M_PI;

    // --- 7. Normalize to 0–180 ---
    if (theta1_deg < 0) theta1_deg += 180;
    if (theta2_deg < 0) theta2_deg += 180;

    // --- 8. Apply calibration offsets (home position = 90,83) ---
    int const servo1 = constrain(round(theta1_deg), 0, 180);
    int const servo2 = constrain(round(theta2_deg), 0, 180);

    Serial.printf(
        "cos_beta=%.2f  beta=%.2f  phi=%.2f  alpha=%.2f  theta1=%.2f  theta2=%.2f\n",
        cos_beta, beta * 180.0f / M_PI, phi * 180.0f / M_PI, alpha * 180.0f / M_PI,
        theta1_deg, theta2_deg
    );

    return { servo1, servo2 };
}
