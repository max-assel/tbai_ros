#!/usr/bin/env python3

import functools
import time
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
import rospy
import sympy as sp
from geometry_msgs.msg import Point, Twist
from scipy.spatial.transform import Rotation as R
from tbai_safe.cbf import ControlBarrierFunctionFactory, visualize_cbfs
from tbai_safe.mppi import (
    AcceleratedSafetyMPPI,
    MppiCbfCost,
    MppiCbfCostInputs,
    cost_fn,
    get_cost_function_parameterized,
)
from tbai_safe.symperf import jit_expr
from tbai_safe.systems import SimpleSingleIntegrator2D
from visualization_msgs.msg import Marker, MarkerArray

from tbai_ros_msgs.msg import EstimatedState


def extract_yaw(quat_xyzw: np.ndarray) -> float:
    """Extract yaw angle from quaternion, ordered as xyzw"""
    return R.from_quat(quat_xyzw).as_euler("zyx", degrees=False)[0]


def log_cps(fn: Callable, log_interval: float = 5.0) -> Callable:
    """Log the number of calls per second of a function, log every `log_interval` seconds"""
    call_count, last_log_time = 0, None

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        nonlocal call_count, last_log_time

        # Ensure proper initialization of last_log_time
        if last_log_time is None:
            last_log_time = rospy.get_time()

        # Update statistics
        call_count, current_time = call_count + 1, rospy.get_time()

        # Log fps if interval has passed
        if current_time - last_log_time > log_interval:
            fps = call_count / (current_time - last_log_time)
            rospy.loginfo(f"[{fn.__name__}] fps: {fps}")
            last_log_time = current_time
            call_count = 0

        return fn(*args, **kwargs)

    return wrapper


def angle_diff(alpha: float, beta: float) -> float:
    """Compute the difference between two angles, in radians"""
    return (beta - alpha + np.pi) % (2 * np.pi) - np.pi


class SafeNode:
    def __init__(self):
        self.state_subscriber = rospy.Subscriber("/estimated_state", EstimatedState, self._state_callback)
        self.twist_subscriber = rospy.Subscriber("/cmd_vel", Twist, self._twist_callback)
        self.twist_publisher = rospy.Publisher("/cmd_vel_safe", Twist, queue_size=1)
        self.cbf_marker_pub = rospy.Publisher("/cbf_markers", MarkerArray, queue_size=1, latch=True)
        self.trajectory_marker_pub = rospy.Publisher("/trajectory_markers", MarkerArray, queue_size=1)
        self.frame_id = rospy.get_param("~frame_id", "odom")

        self.x = 0
        self.y = 0
        self.z = 0
        self.quat = np.array([0, 0, 0, 1])

        self.cmd_x = 0
        self.cmd_y = 0
        self.rot_z = 0

        self.first_message = False

    def wait_for_first_message(self):
        while not self.first_message:
            rospy.sleep(0.1)
        self.first_message = True

    @functools.partial(log_cps, log_interval=10)
    def _state_callback(self, state):
        base_position = state.base_position
        self.x = base_position[0]
        self.y = base_position[1]
        self.z = base_position[2]
        self.quat = state.base_orientation_xyzw
        self.first_message = True

    @functools.partial(log_cps, log_interval=10)
    def _twist_callback(self, twist):
        self.cmd_x = twist.linear.x
        self.cmd_y = twist.linear.y
        self.rot_z = twist.angular.z
        # self.twist_publisher.publish(twist)

    def _make_cylinder_marker(self, cx, cy, r, z=0.0, height=0.05, marker_id=0, color=(1.0, 0.0, 0.0, 0.3)):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cbf_regions"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(cx)
        marker.pose.position.y = float(cy)
        marker.pose.position.z = float(z + 0.5 * height)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(2.0 * r)
        marker.scale.y = float(2.0 * r)
        marker.scale.z = float(height)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def _make_circle_boundary_marker(
        self, cx, cy, r, z=0.02, marker_id=1000, color=(1.0, 0.2, 0.2, 0.9), thickness=0.03, resolution=64
    ):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cbf_boundaries"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(thickness)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for i in range(resolution + 1):
            theta = 2.0 * np.pi * float(i) / float(resolution)
            px = cx + r * np.cos(theta)
            py = cy + r * np.sin(theta)
            p = Point()
            p.x = float(px)
            p.y = float(py)
            p.z = float(z)
            marker.points.append(p)
        marker.lifetime = rospy.Duration(0.0)
        return marker

    def publish_cbf_markers(self, spheres, z=0.0, height=2.05):
        # spheres: list of dicts with keys: cx, cy, r, id
        marker_array = MarkerArray()
        for idx, s in enumerate(spheres):
            cx, cy, r = float(s["cx"]), float(s["cy"]), float(s["r"])
            base_id = int(s.get("id", idx))
            cyl = self._make_cylinder_marker(cx, cy, r, z=z, height=height, marker_id=base_id)
            boundary = self._make_circle_boundary_marker(cx, cy, r, z=z + height + 0.01, marker_id=base_id + 1000)
            marker_array.markers.append(cyl)
            marker_array.markers.append(boundary)
        self.cbf_marker_pub.publish(marker_array)

    def _make_trajectory_marker(self, trajectory, z=0.05, marker_id=2000, color=(0.0, 1.0, 0.0, 0.8), thickness=0.05):
        """Create a LINE_STRIP marker for the optimal trajectory"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "optimal_trajectory"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(thickness)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Add trajectory points
        for point in trajectory:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(z)
            marker.points.append(p)

        marker.lifetime = rospy.Duration(0.1)  # Short lifetime for dynamic updates
        return marker

    def _make_trajectory_points_marker(
        self, trajectory, z=0.05, marker_id=2001, color=(0.0, 0.8, 1.0, 0.9), point_size=0.03
    ):
        """Create SPHERE_LIST marker for trajectory waypoints"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(point_size)
        marker.scale.y = float(point_size)
        marker.scale.z = float(point_size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Add trajectory points
        for point in trajectory:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(z)
            marker.points.append(p)

        marker.lifetime = rospy.Duration(0.1)  # Short lifetime for dynamic updates
        return marker

    def publish_trajectory_markers(self, trajectory, z=0.05):
        """Publish trajectory markers for RViz visualization"""
        if trajectory is None or len(trajectory) == 0:
            return

        marker_array = MarkerArray()

        # Create line strip for the trajectory path
        line_marker = self._make_trajectory_marker(trajectory, z=z, marker_id=2000)
        marker_array.markers.append(line_marker)

        # Create points for trajectory waypoints
        points_marker = self._make_trajectory_points_marker(trajectory, z=z, marker_id=2001)
        marker_array.markers.append(points_marker)

        self.trajectory_marker_pub.publish(marker_array)


def main():
    rospy.init_node("tbai_safe_node")
    safe_node = SafeNode()
    print("Waiting for first message...")
    safe_node.wait_for_first_message()
    print("First message received.")

    rate = rospy.Rate(10)
    fig, ax = plt.subplots()
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Base X-Y Position")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.legend()
    plt.show(block=False)

    last_angle_diff = None

    # Initialize system
    system = SimpleSingleIntegrator2D().reset(
        np.array([safe_node.x, safe_node.y]), visualizer=(fig, ax), visualize_history=True
    )
    system.visualize()
    dt = 0.02

    factory2 = ControlBarrierFunctionFactory()

    d = 1.8
    cbf10 = factory2.get_sphere().substitute(c_x=0.5, c_y=-d, r=1.3)
    cbf11 = factory2.get_sphere().substitute(c_x=-0.5, c_y=d, r=1.3)
    cbf12 = factory2.get_sphere().substitute(c_x=-1.6, c_y=0, r=1.3)
    cbf10_jit = jit_expr(cbf10.get_expr(substitute=True))
    cbf11_jit = jit_expr(cbf11.get_expr(substitute=True))
    cbf12_jit = jit_expr(cbf12.get_expr(substitute=True))
    pcm = visualize_cbfs([cbf10, cbf11, cbf12], ax, granularity=200, unsafe_colors=["red" for _ in range(3)], alpha=0.9)

    cbf_params = [
        {"cx": 0.5, "cy": -d, "r": 1.3, "id": 0},
        {"cx": -0.5, "cy": d, "r": 1.3, "id": 1},
        {"cx": -1.6, "cy": 0.0, "r": 1.3, "id": 2},
    ]

    # Publish CBF markers in RViz
    safe_node.publish_cbf_markers(cbf_params, z=safe_node.z, height=0.05)

    # Get LQR controller
    Q, R = np.eye(2), np.eye(2)

    # Prepare stage cost
    x1, x2, u1, u2 = factory2.x, factory2.y, factory2.u1, factory2.u2
    x_desireds, y_desireds = sp.symbols("x_desired, y_desired")
    lqr_stage_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desireds, y_desireds, 0.0, 0.0)
    lqr_stage_jit = jit_expr(lqr_stage_cost_expr)

    @functools.partial(cost_fn, locals=locals(), globals=globals())
    def stage_cost(x, y, u1, u2, weight1, weight2, weight3, weight4, alpha, x_desired, y_desired):
        cbf10_val = cbf10_jit(x=x, y=y)
        cbf11_val = cbf11_jit(x=x, y=y)
        cbf12_val = cbf12_jit(x=x, y=y)
        cost = 0.0
        cost += (-weight1 * cbf10_val) if cbf10_val < 0 else 0
        cost += (-weight2 * cbf11_val) if cbf11_val < 0 else 0
        cost += (-weight3 * cbf12_val) if cbf12_val < 0 else 0
        abs_cost = -cost if cost < 0 else cost
        if abs_cost == 0:
            abs_cost = 1.0
        abs_cost = max(abs_cost, 1.0)
        cost += (alpha * lqr_stage_jit(x=x, y=y, u1=0.0, u2=0.0, x_desired=x_desired, y_desired=y_desired)) / abs_cost
        return cost

    lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desireds, y_desireds, 0.0, 0.0)
    lqr_final_jit = jit_expr(lqr_final_cost_expr)

    @functools.partial(cost_fn, locals=locals(), globals=globals())
    def terminal_cost(x, y, weight1, weight2, weight3, weight4, alpha, x_desired, y_desired):
        return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0, x_desired=x_desired, y_desired=y_desired)

    mppi_cost_fn = get_cost_function_parameterized(
        stage_cost,
        terminal_cost,
        scalar_args=["weight1", "weight2", "weight3", "weight4"],
        vector_args_ew=["alpha", "x_desired", "y_desired"],
        vector_args_vw=[],
    )

    cost = MppiCbfCost(mppi_cost_fn)

    print("Jitted!")

    mppi = AcceleratedSafetyMPPI(
        system=system,
        lqr_Q=np.eye(2),
        lqr_R=np.eye(2),
        dt=0.02,
        horizon=20,
        mc_rollouts=50000,
        lmbda=50.0,
        sigma=np.eye(2) * 4.0,
        transition_time=20,
        x_desired=np.array([0, 0]),
        return_sampled_trajectories=False,
        return_optimal_trajectory=True,
    )

    # Visualize initial and desired states
    ax.plot(safe_node.x, safe_node.y, "rx", label="Initial state", markersize=10)
    (desired_plot,) = ax.plot(0, 0, "bx", label="Desired state", markersize=10)
    ax.legend()

    # Get LQR controller
    Q, R = np.eye(2), np.eye(2)

    T = 10
    dt = 0.08

    # Add an arrow to show the robot's orientation based on yaw
    arrow = None  # Initialize arrow variable outside the loop if needed

    (optimal_trajectory_plot,) = ax.plot([], [], "k-", label="Optimal trajectory")

    last_cbf_publish_time = time.time()

    while not rospy.is_shutdown():
        system.set_state(np.array([safe_node.x, safe_node.y]))
        system.visualize()
        fig.canvas.draw()
        fig.canvas.flush_events()
        vx, vy, wz = safe_node.cmd_x, safe_node.cmd_y, safe_node.rot_z

        x, y = safe_node.x, safe_node.y

        # Forward rollout from
        yaw = extract_yaw(safe_node.quat)

        cy, sy = np.cos(yaw), np.sin(yaw)
        transform = True
        if transform:
            vx_w, vy_w = vx * cy - vy * sy, vx * sy + vy * cy
        else:
            vx_w, vy_w = vx, vy
        yaw_w = yaw

        x_hist, y_hist = [x], [y]
        for _ in range(T):
            x += vx_w * dt
            y += vy_w * dt
            cy, sy = np.cos(yaw_w), np.sin(yaw_w)
            yaw_w += wz * dt * 5
            if transform:
                vx_w, vy_w = vx * cy - vy * sy, vx * sy + vy * cy
            else:
                vx_w, vy_w = vx, vy
            x_hist.append(float(x))
            y_hist.append(float(y))
        desired_plot.set_data([x_hist[-1]], [y_hist[-1]])
        # target_plot.set_data(x_hist, y_hist)

        if time.time() - last_cbf_publish_time > 3:
            safe_node.publish_cbf_markers(cbf_params, z=safe_node.z, height=0.05)
            last_cbf_publish_time = time.time()
            rospy.loginfo("Published CBF markers")

        # Remove previous arrow if it exists
        if arrow is not None:
            arrow.remove()
            arrow = None

        # Draw an arrow at the robot's position in the direction of yaw
        arrow_length = 0.3  # Length of the arrow
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        arrow = ax.arrow(
            safe_node.x, safe_node.y, dx, dy, head_width=0.08, head_length=0.12, fc="g", ec="g", linewidth=2, zorder=5
        )

        t1 = time.time()
        control, _, optimal_trajectory, st = mppi.calc_control_input(
            system.state,
            [
                (
                    cost,
                    MppiCbfCostInputs(
                        scalars=[5000.0, 5000.0, 5000.0, 5000.0],
                        vectors_ew=[
                            mppi.relaxation_alphas,
                            [float(x_hist[-1]) for _ in range(mppi.T)],
                            [float(y_hist[-1]) for _ in range(mppi.T)],
                        ],
                        vectors_vw=[],
                    ),
                ),
            ],
            x_desired=np.array([x_hist[-1], y_hist[-1]]),
        )
        desired_plot.set_data([x_hist[-1]], [y_hist[-1]])
        t2 = time.time()

        vx_desired, vy_desired = control[0], control[1]
        vx_desired_b, vy_desired_b = vx_desired * cy + vy_desired * sy, -vx_desired * sy + vy_desired * cy

        desired_yaw = np.arctan2(control[1], control[0])

        command_twist = Twist()
        command_twist.linear.x = vx_desired_b * 0.999
        command_twist.linear.y = vy_desired_b * 0.999
        follow = np.linalg.norm([vx_desired_b, vy_desired_b]) >= 0.2

        angle_diff_front = angle_diff(yaw, desired_yaw)
        angle_diff_back = angle_diff(yaw, desired_yaw + np.pi)

        # Pick the smaller angle difference (front or back) and set follow accordingly
        if abs(angle_diff_front) < abs(angle_diff_back):
            chosen_angle_diff = angle_diff_front
        else:
            chosen_angle_diff = angle_diff_back

        follow = 0
        command_twist.angular.z = np.clip(0.3 * (chosen_angle_diff * follow) + (1 - follow) * wz, -0.6, 0.6)
        safe_node.twist_publisher.publish(command_twist)

        if mppi.return_optimal_trajectory:
            optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])
            safe_node.publish_trajectory_markers(optimal_trajectory, z=safe_node.z + 0.02)

        rate.sleep()
    plt.show()


if __name__ == "__main__":
    main()
