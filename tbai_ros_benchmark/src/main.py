#!/usr/bin/env python3

import argparse

import rospy
from geometry_msgs.msg import Twist
from helpers import (
  EndEffectorKinematics,
  PinocchioInterface,
  SlipDetector,
  StateSubscriber,
  Statistician,
  TrackFollower,
  TrackModel,
)
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--context_aware",
    action="store_true",
    help="Detect slips and change to the RL controller when needed",
  )
  parser.add_argument("--start_waypoint", type=int, default=1, help="Start waypoint")
  return parser.parse_args()


def main():
  args = parse_args()
  rospy.init_node("tbai_ros_benchmark")
  rospy.loginfo("Context-aware controller" + "enabled" if args.context_aware else "disabled")

  robot_name = rospy.get_param("robot_name")

  ## setup
  urdf = rospy.get_param("/robot_description")
  foot_names = ["LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"]
  pinocchio_interface = PinocchioInterface(urdf)
  end_effector_kinematics = EndEffectorKinematics(
    pinocchio_interface.get_model(), pinocchio_interface.get_data(), foot_names
  )

  ## Benchmark track
  track = TrackModel.benchmark_track(args.start_waypoint)

  ## Autonomous controller
  track_follower = TrackFollower(track)

  ## Slip detector
  slip_detector = SlipDetector(3.8, pinocchio_interface, end_effector_kinematics)

  ## Phase, start, all that
  phase = 0.0

  ## Statistician
  statistician = Statistician(track, phase)

  ## State subscriber and twist publisher
  state_topic = f"/{robot_name}/state"
  command_topic = "/cmd_vel"
  state_subscriber = StateSubscriber(state_topic)
  twist_publisher = rospy.Publisher(command_topic, Twist, queue_size=1)
  change_publisher = rospy.Publisher(f"/{robot_name}/change_controller", String, queue_size=1)

  ## Main loop
  rate = rospy.Rate(10)
  controller = "WBC"
  while not rospy.is_shutdown():
    # rospy.loginfo("Running")
    rate.sleep()

    # Get new twist command
    xyz, yaw = state_subscriber.get_xyz_yaw()
    twist = track_follower.run(xyz, yaw, phase)
    # print(yaw, phase)
    twist_publisher.publish(twist)

    ## Detect slips
    q, v, contacts = state_subscriber.get_pin_and_contacts()
    slips = slip_detector.detect_slips(q, v, contacts)

    change_controller = any(slips) and args.context_aware

    if change_controller:
      print("Slips detected, changing to RL controller")
      change_publisher.publish(String("RL"))
      controller = "RL"

    # Update phase
    phase = track_follower.update_phase(xyz, phase)

    # Print stats
    wp_reached = statistician.update(phase)

    if wp_reached and controller == "RL":
      print("Waypoint reached, changing to WBC controller")
      # Stop and change to WBC
      for i in range(3):
        twist_publisher.publish(Twist())
        rate.sleep()
      change_publisher.publish(String("WBC"))
      controller = "WBC"

    # print(phase)


if __name__ == "__main__":
  main()
