#!/usr/bin/env python3

import rospy
import argparse

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from helpers import (
    PinocchioInterface,
    EndEffectorKinematics,
    TrackFollower,
    TrackModel,
    StateSubscriber,
    SlipDetector,
    Statistician,
)


def parse_args() -> argparse.Namespace:
    """Parse the command-line arguments used to configure the benchmark.

    Returns
    -------
    argparse.Namespace
        A namespace containing the parsed arguments. The namespace always
        contains the following attributes:

        context_aware : bool
            If *True* the benchmark will switch to an RL-based controller
            whenever excessive foot-slip is detected.
        start_waypoint : int
            Index (1-based) of the first waypoint of the benchmark track
            from which the evaluation should start.
    """
    parser = argparse.ArgumentParser(description="TBAI locomotion benchmark runner")
    parser.add_argument(
        "--context_aware",
        action="store_true",
        help="Detect slips and change to the RL controller when needed",
    )
    parser.add_argument("--start_waypoint", type=int, default=1, help="Start waypoint")
    return parser.parse_args()


def main():
    """Entry-point for the *tbai_ros_benchmark* node.

    This function wires together all helper utilities defined in
    `helpers.py` and starts the main ROS control loop that repeatedly:

    1. Computes a desired twist command that follows the predefined track.
    2. Publishes the command on */cmd_vel*.
    3. Monitors foot slips and, if *context_aware* is enabled, switches to
       an RL controller when a slip is detected.
    4. Collects simple statistics such as the time required to reach the
       next waypoint.
    """
    args = parse_args()
    rospy.init_node("tbai_ros_benchmark")
    rospy.loginfo("Context-aware controller" + "enabled" if args.context_aware else "disabled")

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
    state_topic = "/anymal_d/state"
    command_topic = "/cmd_vel"
    state_subscriber = StateSubscriber(state_topic)
    twist_publisher = rospy.Publisher(command_topic, Twist, queue_size=1)
    change_publisher = rospy.Publisher("/anymal_d/change_controller", String, queue_size=1)

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
