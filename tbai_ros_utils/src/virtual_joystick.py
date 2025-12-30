import math
import tkinter as tk
from tkinter import ttk

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty


class VirtualJoystick(tk.Frame):
  def __init__(self, parent, size=200, **kwargs):
    super().__init__(parent, **kwargs)
    self.size = size
    self.center = size // 2
    self.max_distance = size // 2 - 10

    self.canvas = tk.Canvas(self, width=size, height=size, bg="lightgray")
    self.canvas.pack(padx=10, pady=10)

    self.canvas.create_oval(5, 5, size - 5, size - 5, outline="black", width=2)

    self.knob_size = 20
    self.knob_x = self.center
    self.knob_y = self.center
    self.knob = self.canvas.create_oval(
      self.knob_x - self.knob_size // 2,
      self.knob_y - self.knob_size // 2,
      self.knob_x + self.knob_size // 2,
      self.knob_y + self.knob_size // 2,
      fill="red",
      outline="darkred",
      width=2,
    )

    self.canvas.bind("<Button-1>", self.on_click)
    self.canvas.bind("<B1-Motion>", self.on_drag)
    self.canvas.bind("<ButtonRelease-1>", self.on_release)

    self.command_callback = None

    self.current_x = 0.0
    self.current_y = 0.0
    self.current_z = 0.0

  def set_command_callback(self, callback):
    """Set callback function to be called when joystick position changes"""
    self.command_callback = callback

  def on_click(self, event):
    self.move_knob(event.x, event.y)

  def on_drag(self, event):
    self.move_knob(event.x, event.y)

  def on_release(self, event):
    # Return to center
    self.move_knob(self.center, self.center)

  def move_knob(self, x, y):
    dx = x - self.center
    dy = y - self.center
    distance = math.sqrt(dx * dx + dy * dy)

    if distance > self.max_distance:
      angle = math.atan2(dy, dx)
      x = self.center + self.max_distance * math.cos(angle)
      y = self.center + self.max_distance * math.sin(angle)
      dx = x - self.center
      dy = y - self.center
      distance = self.max_distance

    self.knob_x = x
    self.knob_y = y
    self.canvas.coords(
      self.knob,
      x - self.knob_size // 2,
      y - self.knob_size // 2,
      x + self.knob_size // 2,
      y + self.knob_size // 2,
    )

    self.current_x = dx / self.max_distance
    self.current_y = -dy / self.max_distance
    self.current_z = 0.0

    if self.command_callback:
      self.command_callback(self.current_x, self.current_y, self.current_z)


class UIController:
  def __init__(self, robot_name):
    self.root = tk.Tk()
    self.root.title("Virtual Joystick")
    self.root.resizable(False, False)  # Prevent resizing
    self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    main_frame = ttk.Frame(self.root, padding="10")
    main_frame.pack()

    joystick_frame = ttk.Frame(main_frame)
    joystick_frame.pack()

    left_frame = ttk.Frame(joystick_frame)
    left_frame.grid(row=0, column=0, padx=15)

    left_title = ttk.Label(left_frame, text="X+Y Velocity", font=("Arial", 12, "bold"))
    left_title.pack(pady=(0, 5))

    self.left_joystick = VirtualJoystick(left_frame, size=160)
    self.left_joystick.pack()
    self.left_joystick.set_command_callback(self.publish_xy_velocity)

    # Right joystick frame (X+Yaw Velocity)
    right_frame = ttk.Frame(joystick_frame)
    right_frame.grid(row=0, column=1, padx=15)

    right_title = ttk.Label(right_frame, text="X+Yaw Velocity", font=("Arial", 12, "bold"))
    right_title.pack(pady=(0, 5))

    self.right_joystick = VirtualJoystick(right_frame, size=160)
    self.right_joystick.pack()
    self.right_joystick.set_command_callback(self.publish_x_yaw_velocity)

    button_frame = ttk.Frame(main_frame)
    button_frame.pack(pady=(20, 0))

    button_title = ttk.Label(button_frame, text="Controller", font=("Arial", 12, "bold"))
    button_title.pack(pady=(0, 10))

    button_grid = ttk.Frame(button_frame)
    button_grid.pack()

    row1_frame = ttk.Frame(button_grid)
    row1_frame.pack()

    self.stand_button = ttk.Button(row1_frame, text="STAND", command=lambda: self.publish_controller_change("STAND"))
    self.stand_button.pack(side=tk.LEFT, padx=5)

    self.sit_button = ttk.Button(row1_frame, text="SIT", command=lambda: self.publish_controller_change("SIT"))
    self.sit_button.pack(side=tk.LEFT, padx=5)

    self.bob_button = ttk.Button(row1_frame, text="BOB", command=lambda: self.publish_controller_change("BOB"))
    self.bob_button.pack(side=tk.LEFT, padx=5)

    self.wbc_button = ttk.Button(row1_frame, text="MPC+WBC", command=lambda: self.publish_controller_change("WBC"))
    self.wbc_button.pack(side=tk.LEFT, padx=5)

    self.np3o_button = ttk.Button(row1_frame, text="NP3O", command=lambda: self.publish_controller_change("NP3O"))
    self.np3o_button.pack(side=tk.LEFT, padx=5)

    self.joe_button = ttk.Button(row1_frame, text="JOE", command=lambda: self.publish_controller_change("JOE"))
    self.joe_button.pack(side=tk.LEFT, padx=5)

    self.dtc_button = ttk.Button(row1_frame, text="DTC", command=lambda: self.publish_controller_change("DTC"))
    self.dtc_button.pack(side=tk.LEFT, padx=5)

    # Second row for additional buttons
    row2_frame = ttk.Frame(button_grid)
    row2_frame.pack(pady=(5, 0))

    self.clear_map_button = ttk.Button(row2_frame, text="Clear Map", command=self.clear_elevation_map)
    self.clear_map_button.pack(side=tk.LEFT, padx=5)

    self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    self.controller_pub = rospy.Publisher(f"/{robot_name}/change_controller", String, queue_size=10)

    self.linear_x = 0.0
    self.linear_y = 0.0
    self.angular_z = 0.0

    self.linear_scale = 1.0  # m/s
    self.angular_scale = 2.0  # rad/s

    self.root.update_idletasks()

    self.root.after(100, self.update_gui)

  def publish_xy_velocity(self, x, y, z):
    """Handle X+Y velocity from left joystick"""
    self.linear_x = y * self.linear_scale
    self.linear_y = -x * self.linear_scale

    self.publish_combined_twist()

  def publish_x_yaw_velocity(self, x, y, z):
    """Handle X+Yaw velocity from right joystick"""
    self.linear_x = y * self.linear_scale
    self.angular_z = -x * self.angular_scale / 4.0

    self.publish_combined_twist()

  def publish_combined_twist(self):
    """Publish the combined twist message"""
    twist = Twist()
    twist.linear.x = self.linear_x
    twist.linear.y = self.linear_y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = self.angular_z

    self.velocity_pub.publish(twist)

  def publish_controller_change(self, controller_name):
    """Publish controller change command"""
    msg = String()
    msg.data = controller_name
    self.controller_pub.publish(msg)
    rospy.loginfo(f"Published controller change: {controller_name}")

  def clear_elevation_map(self):
    """Call ROS service to clear elevation map"""
    try:
      rospy.wait_for_service("/elevation_mapping/clear_map", timeout=1.0)
      clear_map_service = rospy.ServiceProxy("/elevation_mapping/clear_map", Empty)
      response = clear_map_service()
      rospy.loginfo("Elevation map cleared successfully")
    except rospy.ServiceException as e:
      rospy.logerr(f"Service call failed: {e}")
    except rospy.ROSException as e:
      rospy.logerr(f"Service not available: {e}")

  def update_gui(self):
    """Update GUI periodically"""
    if not rospy.is_shutdown():
      self.root.after(50, self.update_gui)
    else:
      self.root.quit()

  def on_closing(self):
    zero_twist = Twist()
    self.velocity_pub.publish(zero_twist)
    self.root.quit()

  def run(self):
    """Start the GUI main loop"""
    self.root.mainloop()


def main():
  rospy.init_node("virtual_joystick", anonymous=True)
  robot_name = rospy.get_param("/robot_name", "anymal")
  rospy.loginfo(f"Robot name parameter: {robot_name}")

  try:
    ui_controller = UIController(robot_name)
    ui_controller.run()
  except KeyboardInterrupt:
    print("Keyboard interrupt - shutting down")
  except rospy.ROSInterruptException:
    print("ROS interrupt - shutting down")


if __name__ == "__main__":
  main()
