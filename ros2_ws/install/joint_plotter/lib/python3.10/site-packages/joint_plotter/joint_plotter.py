#!/usr/bin/env python3
"""
Sample –x, y and inferred joint angles (deg) from /get_xy, store, then
plot them when you stop the program with Ctrl‑C.
"""

import math
import time
import rclpy
from rclpy.node import Node
from inverse_kinematics_node.srv import GetXY
import matplotlib.pyplot as plt

# ---------- geometric constants (same numbers the IK node uses) -------------
L1 = 30.0          # cm
L2 = 30.0          # cm
BASE_X = -42.0     # cm  (robot base in world frame)
BASE_Y = 0.0       # cm


def cartesian_to_joint(x: float, y: float):
    """Return (j1_deg, j2_deg) or None if point is unreachable."""
    xr = x - BASE_X
    yr = y - BASE_Y
    r2 = xr * xr + yr * yr
    c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c2 < -1.0 or c2 > 1.0:
        return None                     # outside workspace

    theta2 = abs(math.acos(c2))         # elbow‑down
    s2 = math.sin(theta2)
    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = math.atan2(yr, xr) - math.atan2(k2, k1)

    j1_deg = -theta1 * 180.0 / math.pi  # motor sign conventions
    j2_deg =  theta2 * 180.0 / math.pi
    return j1_deg, j2_deg


class JointPlotter(Node):
    def __init__(self):
        super().__init__("joint_plotter")

        # ---- wait for /get_xy service -------------------------------------------------
        self._cli = self.create_client(GetXY, "get_xy")
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_xy service …")
        self.get_logger().info("/get_xy service available – starting sampling")

        # storage: (t, x, y, j1, j2)
        self._buffer = []

        # sample every 0.10 s
        self._timer = self.create_timer(0.10, self._sample_once)

    # -------------------------------------------------------------------------
    def _sample_once(self):
        req = GetXY.Request()
        future = self._cli.call_async(req)
        future.add_done_callback(self._receive)

    # -------------------------------------------------------------------------
    def _receive(self, future):
        if future.result() is None:
            self.get_logger().warn("Service call failed")
            return

        res = future.result()
        joint = cartesian_to_joint(res.x, res.y)
        if joint is None:            # unreachable – skip but warn once/sec
            self.get_logger().warn_throttle(1.0,
                f"({res.x:.1f},{res.y:.1f}) outside workspace ‑ skipped")
            return

        j1, j2 = joint
        now = time.time()
        self._buffer.append((now, res.x, res.y, j1, j2))
        self.get_logger().debug(
            f"#{len(self._buffer):04d}:  x={res.x:.1f}  y={res.y:.1f}  "
            f"j1={j1:.1f}°  j2={j2:.1f}°")

    # -------------------------------------------------------------------------
    def plot(self):
        if not self._buffer:
            print("No data collected; nothing to plot.")
            return

        t0 = self._buffer[0][0]
        t   = [row[0] - t0 for row in self._buffer]
        xs  = [row[1] for row in self._buffer]
        ys  = [row[2] for row in self._buffer]
        j1s = [row[3] for row in self._buffer]
        j2s = [row[4] for row in self._buffer]

        # end‑effector position
        plt.figure()
        plt.plot(t, xs, label="X (cm)")
        plt.plot(t, ys, label="Y (cm)")
        plt.title("End‑effector position")
        plt.xlabel("time (s)"); plt.ylabel("position (cm)")
        plt.grid(True); plt.legend()

        # joint angles
        plt.figure()
        plt.plot(t, j1s, label="Shoulder j1 (°)")
        plt.plot(t, j2s, label="Elbow j2 (°)")
        plt.title("Joint angles")
        plt.xlabel("time (s)"); plt.ylabel("angle (°)")
        plt.grid(True); plt.legend()

        plt.show()


# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = JointPlotter()
    try:
        rclpy.spin(node)        # Ctrl‑C to stop and plot
    except KeyboardInterrupt:
        pass

    # stop timer cleanly
    node.destroy_timer(node._timer)
    node.plot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

