import numpy as np
import matplotlib.pyplot as plt

L1 = 6.0
L2 = 4.7
L3 = 0.1
L4 = 2.8

def degrees_to_radians(deg):
    return np.deg2rad(deg)

def forward_kinematics(theta1_deg, theta2_deg, plot=True):
    theta1 = degrees_to_radians(theta1_deg)
    theta2 = degrees_to_radians(theta2_deg)

    # Position of first joint (elbow)
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    # Position of end of second link (before tool offset)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    # Apply tool offset (L4) in direction of end-effector
    x_end = x2 + L4 * np.cos(theta1 + theta2)
    y_end = y2 + L4 * np.sin(theta1 + theta2)

    if plot:
        plt.figure()
        plt.plot([0, x1, x2, x_end], [0, y1, y2, y_end], "-o", lw=2)
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"FK: θ1={theta1_deg:.1f}°, θ2={theta2_deg:.1f}°")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        plt.show()

    return x_end, y_end

if __name__ == "__main__":
    test_angles = [
        (97, 156),
        (36, 144),
        (0, 156),
        (46, 173),
    ]

    for t1, t2 in test_angles:
        x, y = forward_kinematics(t1, t2, plot=False)
        print(f"θ1={t1}°, θ2={t2}° -> FK = ({x:.2f}, {y:.2f})")

    forward_kinematics(46, 173)