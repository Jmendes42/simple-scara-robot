import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from math import cos, sin, acos, atan2, sqrt, degrees, radians

# === Arm Geometry ===
L1 = 6.0  # upper arm (cm)
L2 = 5.0  # forearm (cm)

# === Calibration Settings ===
theta1_offset_deg = 0    # offset so that θ1=0° points upward
theta2_offset_deg = 0
invert_theta1 = False      # set True if base servo goes opposite
invert_theta2 = True       # set True if elbow servo goes opposite

# === Forward Kinematics ===
def fk(theta1_deg, theta2_deg):
    theta1 = radians(theta1_deg + theta1_offset_deg)
    theta2 = radians(theta2_deg + theta2_offset_deg)
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return x, y

# === Inverse Kinematics ===
def ik(x_target, y_target):
    r2 = x_target**2 + y_target**2
    r = sqrt(r2)
    cos_beta = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = acos(cos_beta)

    theta2 = degrees(beta)
    if invert_theta2:
        theta2 = -theta2

    k1 = L1 + L2 * cos(beta)
    k2 = L2 * sin(beta)
    alpha = atan2(y_target, x_target)
    theta1 = degrees(alpha - atan2(k2, k1))

    if invert_theta1:
        theta1 = -theta1

    theta1 -= theta1_offset_deg
    theta2 -= theta2_offset_deg

    return theta1, theta2

# === Interactive Visualization ===
class ArmSimulator:
    def __init__(self):
        self.target = np.array([6.0, 4.0])
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        plt.subplots_adjust(bottom=0.2)
        self.ax.set_xlim(-12, 12)
        self.ax.set_ylim(-12, 12)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.ax.set_title("Drag the red X to move target")

        self.line, = self.ax.plot([], [], '-o', lw=3, color='blue')
        self.target_point, = self.ax.plot(self.target[0], self.target[1], 'rx', markersize=12, mew=2)
        self.text = self.ax.text(-11, 11, '', fontsize=10, verticalalignment='top')

        self.dragging = False
        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        reset_ax = plt.axes([0.4, 0.05, 0.2, 0.075])
        self.btn_reset = Button(reset_ax, 'Reset Target')
        self.btn_reset.on_clicked(self.reset_target)

        self.update_plot()

    def reset_target(self, event):
        self.target = np.array([6.0, 4.0])
        self.update_plot()

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        if np.hypot(event.xdata - self.target[0], event.ydata - self.target[1]) < 0.6:
            self.dragging = True

    def on_release(self, event):
        self.dragging = False

    def on_motion(self, event):
        if self.dragging and event.inaxes == self.ax:
            self.target = np.array([event.xdata, event.ydata])
            self.update_plot()

    def update_plot(self):
        x_t, y_t = self.target
        theta1, theta2 = ik(x_t, y_t)
        x0, y0 = 0, 0
        x1 = L1 * cos(radians(theta1 + theta1_offset_deg))
        y1 = L1 * sin(radians(theta1 + theta1_offset_deg))
        x2 = x1 + L2 * cos(radians(theta1 + theta1_offset_deg + theta2 + theta2_offset_deg))
        y2 = y1 + L2 * sin(radians(theta1 + theta1_offset_deg + theta2 + theta2_offset_deg))

        self.line.set_data([x0, x1, x2], [y0, y1, y2])
        self.target_point.set_data([x_t], [y_t])
        self.text.set_text(f"Target=({x_t:.2f}, {y_t:.2f})\nθ1={theta1:.1f}°\nθ2={theta2:.1f}°")

        self.fig.canvas.draw_idle()

# === Run simulator ===
if __name__ == "__main__":
    ArmSimulator()
    plt.show()