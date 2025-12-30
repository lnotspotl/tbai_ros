#!/usr/bin/env python3

import functools
import time
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from tbai_cbf_mppi.cbf import ControlBarrierFunctionFactory, visualize_cbfs
from tbai_cbf_mppi.control import VanillaSafetyFilterNew
from tbai_cbf_mppi.mppi import (
  AcceleratedSafetyMPPI,
  MppiCbfCost,
  MppiCbfCostInputs,
  cost_fn,
  get_cost_function_parameterized,
)
from tbai_cbf_mppi.symperf import jit_expr
from tbai_cbf_mppi.systems import SimpleSingleIntegrator2D
from visualization_msgs.msg import Marker, MarkerArray

from tbai_ros_msgs.msg import EstimatedState

# rgba
COLORS = {
  "red": [1.0, 0.0, 0.0, 0.3],
  "green": [0.0, 1.0, 0.0, 0.3],
  "blue": [0.0, 0.0, 1.0, 0.3],
  "yellow": [1.0, 1.0, 0.0, 0.3],
}


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
    robot_name = rospy.get_param("robot_name")
    self.state_subscriber = rospy.Subscriber("/estimated_state", EstimatedState, self._state_callback)
    self.twist_subscriber = rospy.Subscriber("/cmd_vel", Twist, self._twist_callback)
    self.twist_publisher = rospy.Publisher("/cmd_vel_safe", Twist, queue_size=1)
    self.cbf_marker_pub = rospy.Publisher("/cbf_markers", MarkerArray, queue_size=1, latch=True)
    self.trajectory_marker_pub = rospy.Publisher("/trajectory_markers", MarkerArray, queue_size=1)
    self.frame_id = rospy.get_param("~frame_id", "odom")

    self.cbf_switch_subscriber = rospy.Subscriber(f"/{robot_name}/cbf_switch", Bool, self._cbf_switch_callback)
    self.toggle_autonomy_subscriber = rospy.Subscriber(f"/{robot_name}/autonomy", Bool, self._toggle_autonomy_callback)

    self.x = 0
    self.y = 0
    self.z = 0
    self.quat = np.array([0, 0, 0, 1])

    self.cmd_x = 0
    self.cmd_y = 0
    self.rot_z = 0

    self.first_message = False
    self.switch_cbf = False
    self.toggle_autonomy = False

  def _cbf_switch_callback(self, msg):
    self.switch_cbf = True

  def _toggle_autonomy_callback(self, msg):
    self.toggle_autonomy = msg.data

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

  def _make_rectangle_marker(self, cx, cy, w, h, z=0.0, height=0.05, marker_id=0, color=(1.0, 0.0, 0.0, 0.3)):
    marker = Marker()
    marker.header.frame_id = self.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cbf_regions"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = float(cx)
    marker.pose.position.y = float(cy)
    marker.pose.position.z = float(z + 0.5 * height)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(w)
    marker.scale.y = float(h)
    marker.scale.z = float(height)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.lifetime = rospy.Duration(0.0)
    return marker

  def _make_rectangle_boundary_marker(
    self, cx, cy, w, h, z=0.02, marker_id=1000, color=(1.0, 0.2, 0.2, 0.9), thickness=0.03
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

    # Rectangle corners (no rotation)
    half_w = 0.5 * w
    half_h = 0.5 * h
    corners = [
      (cx - half_w, cy - half_h),
      (cx + half_w, cy - half_h),
      (cx + half_w, cy + half_h),
      (cx - half_w, cy + half_h),
      (cx - half_w, cy - half_h),  # Close the loop
    ]
    for px, py in corners:
      p = Point()
      p.x = float(px)
      p.y = float(py)
      p.z = float(z)
      marker.points.append(p)
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

  def publish_cbf_markers(self, cbfs, z=0.0, height=2.05):
    # cbfs: list of dicts with keys: cx, cy, r, id
    marker_array = MarkerArray()
    for idx, (key, value) in enumerate(cbfs.items()):
      if "rect" in key:
        cx, cy, w, h = float(value["c_x"]), float(value["c_y"]), float(value["w"]), float(value["h"])
      elif "sphere" in key:
        cx, cy, r = float(value["c_x"]), float(value["c_y"]), float(value["r"])
      else:
        raise ValueError(f"Unknown CBF type: {key}")
      base_id = int(value.get("id", idx))
      if "rect" in key:
        cyl = self._make_rectangle_marker(
          cx, cy, w, h, z=z, height=height, marker_id=base_id, color=COLORS[value["color"]]
        )
        boundary = self._make_rectangle_boundary_marker(cx, cy, w, h, z=z + height + 0.01, marker_id=base_id + 1000)
      elif "sphere" in key:
        cyl = self._make_cylinder_marker(cx, cy, r, z=z, height=height, marker_id=base_id, color=COLORS[value["color"]])
        boundary = self._make_circle_boundary_marker(
          cx, cy, r, z=z + height + 0.01, marker_id=base_id + 1000, resolution=128
        )
      marker_array.markers.append(cyl)
      marker_array.markers.append(boundary)
    self.cbf_marker_pub.publish(marker_array)

  def _make_coordinate_frame_marker(self, position, orientation_yaw=0.0, z=0.05, marker_id=2000, scale=0.1):
    """Create coordinate frame markers (X, Y, Z axes) at a given position"""
    markers = []

    # Define axis directions and colors
    axes = [
      {"axis": np.array([1.0, 0.0, 0.0]), "color": COLORS["red"][:3] + [0.99]},  # X-axis (red)
      {"axis": np.array([0.0, 1.0, 0.0]), "color": COLORS["green"][:3] + [0.99]},  # Y-axis (green)
      {"axis": np.array([0.0, 0.0, 1.0]), "color": COLORS["blue"][:3] + [0.99]},  # Z-axis (blue)
    ]

    # Rotation matrix for yaw
    cos_yaw, sin_yaw = np.cos(orientation_yaw), np.sin(orientation_yaw)
    rotation_matrix = np.array([[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0], [0, 0, 1]])

    for i, axis_info in enumerate(axes):
      marker = Marker()
      marker.header.frame_id = self.frame_id
      marker.header.stamp = rospy.Time.now()
      marker.ns = "coordinate_frames"
      marker.id = marker_id + i
      marker.type = Marker.ARROW
      marker.action = Marker.ADD

      # Set position
      marker.pose.position.x = float(position[0])
      marker.pose.position.y = float(position[1])
      marker.pose.position.z = float(z)

      # Rotate axis direction by yaw
      rotated_axis = rotation_matrix @ axis_info["axis"]

      # Convert direction to quaternion for arrow orientation
      # Arrow points in +X direction by default, so we need to rotate from [1,0,0] to our desired direction
      from scipy.spatial.transform import Rotation as R

      # Calculate rotation from [1,0,0] to rotated_axis
      if np.allclose(rotated_axis, [1, 0, 0]):
        quat = [0, 0, 0, 1]
      elif np.allclose(rotated_axis, [-1, 0, 0]):
        quat = [0, 0, 1, 0]
      else:
        # General case: rotation from [1,0,0] to rotated_axis
        v1 = np.array([1, 0, 0])
        v2 = rotated_axis
        v = np.cross(v1, v2)
        s = np.linalg.norm(v)
        c = np.dot(v1, v2)

        if s == 0:  # Parallel vectors
          quat = [0, 0, 0, 1]
        else:
          vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
          rotation_mat = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))
          quat = R.from_matrix(rotation_mat).as_quat()  # Returns [x,y,z,w]

      marker.pose.orientation.x = float(quat[0])
      marker.pose.orientation.y = float(quat[1])
      marker.pose.orientation.z = float(quat[2])
      marker.pose.orientation.w = float(quat[3])

      # Set scale
      marker.scale.x = float(scale)  # Arrow length
      marker.scale.y = float(scale * 0.05)  # Arrow width
      marker.scale.z = float(scale * 0.05)  # Arrow height

      # Set color
      marker.color.r, marker.color.g, marker.color.b, marker.color.a = axis_info["color"]

      marker.lifetime = rospy.Duration(0.1)  # Short lifetime for dynamic updates
      markers.append(marker)

    return markers

  def _calculate_trajectory_orientations(self, trajectory):
    """Calculate orientation (yaw) for each point in the trajectory based on movement direction"""
    orientations = []

    for i in range(len(trajectory)):
      if i < len(trajectory) - 1:
        # Calculate direction to next point
        dx = trajectory[i + 1][0] - trajectory[i][0]
        dy = trajectory[i + 1][1] - trajectory[i][1]
        yaw = np.arctan2(dy, dx)
      else:
        # For last point, use same orientation as previous point
        yaw = orientations[-1] if orientations else 0.0

      orientations.append(yaw)

    return orientations

  def publish_trajectory_markers(self, trajectory, z=0.05, frame_scale=0.199, skip_points=1):
    """Publish trajectory markers as coordinate systems for RViz visualization"""
    if trajectory is None or len(trajectory) == 0:
      return

    marker_array = MarkerArray()

    # Calculate orientations for each trajectory point
    orientations = self._calculate_trajectory_orientations(trajectory)

    # Create coordinate frame markers at trajectory points (skip some for clarity)
    marker_id_base = 3000
    for i in range(0, len(trajectory), skip_points + 1):  # Skip points to avoid clutter
      position = trajectory[i]
      orientation_yaw = orientations[i]

      # Create coordinate frame (3 arrows: X, Y, Z)
      frame_markers = self._make_coordinate_frame_marker(
        position=position,
        orientation_yaw=orientation_yaw,
        z=position[2],
        marker_id=marker_id_base + i * 3,
        scale=frame_scale,
      )

      # Add all three axis markers
      for frame_marker in frame_markers:
        marker_array.markers.append(frame_marker)

    self.trajectory_marker_pub.publish(marker_array)


def main():
  rospy.init_node("tbai_safe_node")
  safe_node = SafeNode()
  print("Waiting for first message...")
  safe_node.wait_for_first_message()
  print("First message received.")

  use_mppi = rospy.get_param("~use_mppi", True)

  rate = rospy.Rate(10)

  x_offset = -2.0
  y_offset = -2.0

  # x_desired = np.array([6.0 + x_offset, 6.4 + y_offset])
  x_desired = np.array([2.7, 3.0])

  cbf_params = {
    "rect1": dict(c_x=6 + x_offset, c_y=2.9 + y_offset, w=2.0, h=2.0, kappa=10, color="red"),
    "rect2": dict(c_x=0 + x_offset, c_y=2.9 + y_offset, w=2.0, h=2.0, kappa=10, color="red"),
    "sphere1": dict(c_x=4 + x_offset, c_y=3.4 + y_offset, r=1.3, kappa=10, color="green"),
    "sphere2": dict(c_x=2 + x_offset, c_y=3.6 + y_offset, r=1.3, kappa=10, color="blue"),
  }

  system = SimpleSingleIntegrator2D()
  fig, ax = plt.subplots()
  ax.set_xlabel("X Position")
  ax.set_ylabel("Y Position")
  ax.set_title("Base X-Y Position")
  ax.set_xlim(-1, 8)
  ax.set_ylim(-1, 8)
  ax.legend()
  plt.show(block=False)

  if use_mppi:
    # Strict CBFs
    factory1 = ControlBarrierFunctionFactory()
    cbf1 = factory1.get_rectangle(method="approximate").substitute(c_x=6 + x_offset, c_y=2.9 + y_offset, w=2.0, h=2.0)
    cbf2 = factory1.get_rectangle(method="approximate").substitute(c_x=0 + x_offset, c_y=2.9 + y_offset, w=2.0, h=2.0)
    cbf3 = factory1.union([cbf1, cbf2], method="approximate").substitute(kappa=10)

    # Soft CBFs
    factory2 = ControlBarrierFunctionFactory()
    cbf4 = factory2.get_sphere().substitute(c_x=4 + x_offset, c_y=3.4 + y_offset, r=1.3)
    cbf5 = factory2.get_sphere().substitute(c_x=2 + x_offset, c_y=3.6 + y_offset, r=1.3)

    colors = ["red", "green", "blue"]
    pcm = visualize_cbfs([cbf3, cbf4, cbf5], ax, granularity=200, unsafe_colors=colors, alpha=0.5)

    def eval_cbf(x, y):
      return (cbf4.evaluate(x=x, y=y), cbf5.evaluate(x=x, y=y))

  else:
    # Strict CBFs - group 1
    factory1 = ControlBarrierFunctionFactory()
    cbf11 = factory1.get_rectangle(method="approximate").substitute(**cbf_params["rect1"])
    cbf12 = factory1.get_rectangle(method="approximate").substitute(**cbf_params["rect2"])
    cbf13 = factory1.get_sphere().substitute(**cbf_params["sphere1"])
    cbf14 = factory1.union([cbf11, cbf12, cbf13], method="approximate").substitute(kappa=10)
    safety_filter1 = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf13, alpha=10.5)

    # Strict CBFs - group 2
    factory2 = ControlBarrierFunctionFactory()
    cbf21 = factory2.get_rectangle(method="approximate").substitute(**cbf_params["rect1"])
    cbf22 = factory2.get_rectangle(method="approximate").substitute(**cbf_params["rect2"])
    cbf23 = factory2.get_sphere().substitute(**cbf_params["sphere2"])
    cbf24 = factory2.union([cbf21, cbf22, cbf23], method="approximate").substitute(kappa=10)
    safety_filter2 = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf23, alpha=10.5)

    colors = ["red", "red", "green", "blue"]
    pcm = visualize_cbfs([cbf11, cbf12, cbf13, cbf24], ax, granularity=200, unsafe_colors=colors, alpha=0.5)

    def eval_cbf(x, y):
      return (cbf13.evaluate(x=x, y=y), cbf23.evaluate(x=x, y=y))

    safety_filter = safety_filter2

  # Initialize system
  system = system.reset(np.array([safe_node.x, safe_node.y]), visualizer=(fig, ax), visualize_history=True)
  system.visualize()
  dt = 0.02

  # Get LQR controller
  Q, R = np.eye(2), np.eye(2)

  if use_mppi:
    # Get safety filter
    safety_filter = VanillaSafetyFilterNew(system.get_A(), system.get_B(), cbf3, alpha=10.5)

    # Prepare stage cost
    x1, x2, u1, u2 = factory1.x, factory1.y, factory1.u1, factory1.u2
    lqr_stage_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
    lqr_stage_jit = jit_expr(lqr_stage_cost_expr)
    cbf4_jit = jit_expr(cbf4.get_expr(substitute=True))
    cbf5_jit = jit_expr(cbf5.get_expr(substitute=True))

    @functools.partial(cost_fn, locals=locals(), globals=globals())
    def stage_cost(x, y, u1, u2, weight1, weight2, alpha):
      cost = alpha * lqr_stage_jit(y=y, x=x, u1=u1, u2=u2)

      cbf4_val = cbf4_jit(x=x, y=y)
      cbf5_val = cbf5_jit(x=x, y=y)
      cost += (-weight1 * cbf4_val) if cbf4_val < 0 else 0
      cost += (-weight2 * cbf5_val) if cbf5_val < 0 else 0
      return cost

    lqr_final_cost_expr = system.get_lqr_cost_expr(Q, R, x1, x2, u1, u2, x_desired[0], x_desired[1], 0.0, 0.0)
    lqr_final_jit = jit_expr(lqr_final_cost_expr)

    @functools.partial(cost_fn, locals=locals(), globals=globals())
    def terminal_cost(x, y, weight1=1.0, weight2=1.0, alpha=1.0):
      return lqr_final_jit(x=x, y=y, u1=0.0, u2=0.0)

    mppi_cost_fn = get_cost_function_parameterized(
      stage_cost,
      terminal_cost,
      scalar_args=["weight1", "weight2"],
      vector_args_ew=["alpha"],
    )

    cost = MppiCbfCost(mppi_cost_fn)

    print("Jitted!")

    mppi = AcceleratedSafetyMPPI(
      system=system,
      lqr_Q=np.eye(2),
      lqr_R=np.eye(2),
      dt=0.1,
      horizon=20,
      mc_rollouts=1000,
      lmbda=50.0,
      sigma=np.eye(2) * 4.0,
      transition_time=35,
      x_desired=x_desired,
      return_sampled_trajectories=False,
      return_optimal_trajectory=True,
    )

    sampled_trajectories = []
    for _ in range(1):
      sampled_trajectories.append(ax.plot([], [], "k-")[0])

    (optimal_trajectory_plot,) = ax.plot([], [], "k-", label="Optimal trajectory")
    weight1 = 1.0
    weight2 = 5000.0
    flip, publish_commands = False, False

  K = system.get_lqr_gain(Q=Q, R=R)

  def lqr_command(pos: np.ndarray, pos_desired: np.ndarray):
    return -K @ (pos - pos_desired)

  # Add an arrow to show the robot's orientation based on yaw
  arrow = None  # Initialize arrow variable outside the loop if needed

  (optimal_trajectory_plot,) = ax.plot([], [], "k-", label="Optimal trajectory")

  def on_key_press(event):
    nonlocal safe_node, system, fig, ax
    if event.key == "c":
      safe_node.switch_cbf = True
    if event.key == "p":
      safe_node.toggle_autonomy = not safe_node.toggle_autonomy

  fig.canvas.mpl_connect("key_press_event", on_key_press)

  # Enable grid
  ax.minorticks_on()
  ax.grid(which="both", alpha=0.2)

  fig_cbf, ax_cbf = plt.subplots()
  ax_cbf.set_xlabel("Time")
  ax_cbf.set_ylabel("CBF Value")
  ax_cbf.set_title("CBF Values")
  ax_cbf.legend()

  (cbf1_plot,) = ax_cbf.plot([], [], label="CBF 1")
  (cbf2_plot,) = ax_cbf.plot([], [], label="CBF 2")

  plt.show(block=False)

  safe_node.publish_cbf_markers(cbf_params, height=0.05)

  x_hist, y_hist, z_hist = [], [], []

  cbf1_hist, cbf2_hist, time_hist = [], [], []

  last_cbf_publish_time = rospy.Time.now()

  while not rospy.is_shutdown():
    system.set_state(np.array([safe_node.x, safe_node.y]))

    system.visualize()
    fig.canvas.draw()
    fig.canvas.flush_events()
    vx, vy, wz = safe_node.cmd_x, safe_node.cmd_y, safe_node.rot_z

    x, y = safe_node.x, safe_node.y

    # Forward rollout from
    yaw = extract_yaw(safe_node.quat)

    if safe_node.toggle_autonomy:
      every_nth = 1
      trajectory_points = [(x, y, z) for x, y, z in zip(x_hist[::every_nth], y_hist[::every_nth], z_hist[::every_nth])]
      safe_node.publish_trajectory_markers(trajectory_points)

      if rospy.Time.now() - last_cbf_publish_time > rospy.Duration(1.0):
        safe_node.publish_cbf_markers(cbf_params, z=safe_node.z, height=0.05)
        last_cbf_publish_time = rospy.Time.now()

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

    if safe_node.switch_cbf and not use_mppi:
      safety_filter = safety_filter2 if safety_filter == safety_filter1 else safety_filter1
      colors[1], colors[2] = colors[2], colors[1]
      pcm = visualize_cbfs([cbf11, cbf12, cbf13, cbf24], ax, granularity=200, unsafe_colors=colors, alpha=0.5, pcm=pcm)
      print("Flipped")
      safe_node.switch_cbf = False

    if safe_node.switch_cbf and use_mppi:
      weight1, weight2 = weight2, weight1
      mppi.reset_relaxation()
      colors[1], colors[2] = colors[2], colors[1]
      pcm = visualize_cbfs([cbf3, cbf4, cbf5], ax, granularity=200, unsafe_colors=colors, alpha=0.5, pcm=pcm)
      cbf_params["sphere1"]["color"], cbf_params["sphere2"]["color"] = (
        cbf_params["sphere2"]["color"],
        cbf_params["sphere1"]["color"],
      )
      safe_node.publish_cbf_markers(cbf_params, height=0.05)
      print("Flipped")
      safe_node.switch_cbf = False

    t1 = time.time()
    if use_mppi:
      control, _, optimal_trajectory, st = mppi.calc_control_input(
        system.state,
        [
          (
            cost,
            MppiCbfCostInputs(
              scalars=[weight1, weight2], vectors_ew=[np.array(mppi.relaxation_alphas[: mppi.T])], vectors_vw=[]
            ),
          ),
        ],
      )
      control = safety_filter.solve(state=system.state, u_nominal=control)
    else:
      control = lqr_command(system.state, x_desired)
      control = safety_filter.solve(state=system.state, u_nominal=control)
    t2 = time.time()

    vx_desired, vy_desired = control[0], control[1]
    cy, sy = np.cos(yaw), np.sin(yaw)
    vx_desired_b, vy_desired_b = vx_desired * cy + vy_desired * sy, -vx_desired * sy + vy_desired * cy

    desired_yaw = np.arctan2(control[1], control[0])

    command_twist = Twist()
    vel = np.array([vx_desired_b, vy_desired_b])
    if np.linalg.norm(vel) > 0.5:
      vel = vel / np.linalg.norm(vel) * 0.5
    command_twist.linear.x = vel[0]
    command_twist.linear.y = vel[1]
    follow = np.linalg.norm([vx_desired_b, vy_desired_b]) >= 0.2

    angle_diff_front = angle_diff(yaw, desired_yaw)
    angle_diff_back = angle_diff(yaw, desired_yaw + np.pi)

    # Pick the smaller angle difference (front or back) and set follow accordingly
    if abs(angle_diff_front) < abs(angle_diff_back):
      chosen_angle_diff = angle_diff_front
    else:
      chosen_angle_diff = angle_diff_back
    chosen_angle_diff = angle_diff_front

    command_twist.angular.z = np.clip(0.5 * (chosen_angle_diff * follow) + (1 - follow) * wz, -0.6, 0.6)
    if safe_node.toggle_autonomy:
      safe_node.twist_publisher.publish(command_twist)

    if safe_node.toggle_autonomy and follow:
      x_hist.append(float(safe_node.x))
      y_hist.append(float(safe_node.y))
      z_hist.append(float(safe_node.z))

      cbf1_hist.append(eval_cbf(safe_node.x, safe_node.y)[0])
      cbf2_hist.append(eval_cbf(safe_node.x, safe_node.y)[1])
      time_hist.append(rospy.Time.now().to_sec())

    if use_mppi and mppi.return_optimal_trajectory:
      optimal_trajectory_plot.set_data(optimal_trajectory[:, 0], optimal_trajectory[:, 1])

    cbf1_plot.set_data(time_hist, cbf1_hist)
    cbf2_plot.set_data(time_hist, cbf2_hist)
    ax_cbf.relim()
    ax_cbf.autoscale_view()
    ax_cbf.axhline(0, color="r", linestyle="--", linewidth=1)
    fig_cbf.canvas.draw()
    fig_cbf.canvas.flush_events()

    rate.sleep()
  plt.show()


if __name__ == "__main__":
  main()
