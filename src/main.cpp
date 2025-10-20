
#include <Arduino.h>

#include "InverseKinematics.h"
#include "Joint.h"

uint8_t constexpr joint1Channel = 0;
uint8_t constexpr joint1Pin = 0;
Joint joint1(joint1Channel, joint1Pin);

uint8_t constexpr joint2Channel = 1;
uint8_t constexpr joint2Pin = 1;
Joint joint2(joint2Channel, joint2Pin);

uint8_t constexpr joint3Channel = 2;
uint8_t constexpr joint3Pin = 10;
Joint joint3(joint3Channel, joint3Pin);

void moveTo(float const x, float const y)
{
    auto const jointAngles = InverseKinematics::calculateJointAngles(x, y);
    joint3.setAngle(100);
    joint1.setAngle(jointAngles.theta1);
    joint2.setAngle(jointAngles.theta2);
}



void setup()
{
    Serial.begin(460800);
    joint1.setup();
    joint2.setup();
    joint3.setup();
    delay(500);

    Serial.println("Step 2: Testing IK -> Servo mapping");

    // Test target positions
    float testPoints[][2] = {
        // {0, 10.6},    // forward-right
        // {0, 8},    // straight ahead
        {-6, 4},   // forward-left
        // {0, 6}     // back toward center
    };

    for (auto &p : testPoints) {
        float x = p[0];
        float y = p[1];
        auto angles = InverseKinematics::calculateJointAngles(x, y);
        Serial.print("Target (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(") -> θ1=");
        Serial.print(angles.theta1);
        Serial.print("°, θ2=");
        Serial.println(angles.theta2);

        joint1.setAngle(angles.theta1);
        joint2.setAngle(angles.theta2);
        delay(5000);
    }

    joint1.setAngle(90);
    joint2.setAngle(83);
    Serial.println("Back to home");
}

void loop() {}

