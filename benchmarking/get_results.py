import rosbag
import numpy as np

def is_goal_reached(msg, global_goal):
    current_pose = msg.rbd_state[:6]  # Assuming first 6 elements are orientation and position

    current_position = current_pose[3:6]

    dir_to_goal_odom_frame = np.array(global_goal) - np.array(current_position)

    # Normalize direction vector
    norm = np.linalg.norm(dir_to_goal_odom_frame[0:2]) # Only consider x and y for norm, Z we can assume is fine

    goal_threshold = 0.25
    return norm < goal_threshold

def has_started_moving(msg):
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    movement_threshold = 0.01
    return abs(linear_velocity) > movement_threshold or abs(angular_velocity) > movement_threshold

def accumulate_path_length(current_length, current_state, past_state):
    current_pose = current_state.rbd_state[:6]
    past_pose = past_state.rbd_state[:6]

    current_position = np.array(current_pose[3:6])
    past_position = np.array(past_pose[3:6])

    distance = np.linalg.norm(current_position - past_position)

    return current_length + distance

def has_contact_state_changed(current_state, past_state):
    current_contacts = current_state.contact_flags
    past_contacts = past_state.contact_flags

    contact_change_count = 0
    for leg in current_contacts:
        if (past_contacts[leg] == False and current_contacts[leg] == True):
            contact_change_count += 1

    return contact_change_count


def parse_rosbag():
    bagfilepath = "/home/masselmeier3/tbai_ws/bags/ramp/mpc/0.bag"
    env = "ramp"

    if (env == "balance_beam"):
        global_goal = [0.0, -5.70, 0.575]
    elif (env == "gap_stones"):
        global_goal = [0.0, -4.50, 0.575]
    elif (env == "ramp"):
        global_goal = [0.0, 4.15, 0.785]
    elif (env == "ramped_balance_beam"):
        global_goal = [-8.50, 0.50, 0.575]
    elif (env == "ramped_stepping_stones"):
        global_goal = [-8.0, 7.50, 0.575]     
    elif (env == "rubble"):
        global_goal = [-5.50, 0.0, 0.575]      
    else:
        raise Exception("[GlobalPathVelocityGenerator] Unknown world name")

    # Read only
    time_to_start = None
    time_to_goal = None
    path_length = 0.0
    num_steps = 0
    past_state = None
    goal_state = None
    average_map_dt = None
    num_map_frames = 0
    past_map_time = None
    average_mpc_dt = None
    num_mpc_frames = 0
    past_mpc_time = None
    average_wbc_dt = None
    num_wbc_frames = 0
    past_wbc_time = None
    with rosbag.Bag(bagfilepath, 'r') as bag:

        topics = ["/anymal_d/command", 
                    "/anymal_d/state", 
                    "/anymal_mpc_observation",
                    "/cmd_vel", 
                    "/elevation_mapping/elevation_map_raw", 
                    "/elevation_mapping/elevation_map", 
                    "/convex_plane_decomposition_ros/filtered_map"]

        for topic, msg, t in bag.read_messages(topics=topics):

            if (topic == "/anymal_d/state"):
                # print("state: ", msg)
                current_state = msg

                if (time_to_start is not None):
                    path_length = accumulate_path_length(path_length, current_state, past_state)
                    num_steps += has_contact_state_changed(current_state, past_state)

                goal_reached = is_goal_reached(current_state, global_goal)

                if goal_reached:
                    print("Goal reached!")
                    goal_state = current_state
                    time_to_reach_goal = t.to_sec()
                    print("Time to reach goal: ", time_to_reach_goal)
                    break
                past_state = current_state
            elif (topic == "/cmd_vel"):
                started_moving = has_started_moving(msg)

                if (started_moving and time_to_start is None):
                    time_to_start = t.to_sec()
                    print("Time to start moving: ", time_to_start)
            elif (topic == "/convex_plane_decomposition_ros/filtered_map"):
                current_map_time = t.to_sec()

                if (average_map_dt is None and past_map_time is not None):
                    num_map_frames += 1
                    average_map_dt = current_map_time - past_map_time
                elif (past_map_time is not None):
                    num_map_frames += 1
                    est_map_dt = current_map_time - past_map_time
                    average_map_dt = ((average_map_dt * (num_map_frames - 1)) + est_map_dt) / num_map_frames
                    
                past_map_time = current_map_time
            elif (topic == "/anymal_mpc_observation"):
                current_mpc_time = t.to_sec()

                if (average_mpc_dt is None and past_mpc_time is not None):
                    num_mpc_frames += 1
                    average_mpc_dt = current_mpc_time - past_mpc_time
                elif (past_mpc_time is not None):
                    num_mpc_frames += 1
                    est_mpc_dt = current_mpc_time - past_mpc_time
                    average_mpc_dt = ((average_mpc_dt * (num_mpc_frames - 1)) + est_mpc_dt) / num_mpc_frames
                    
                past_mpc_time = current_mpc_time
            elif (topic == "/anymal_d/command"):
                current_wbc_time = t.to_sec()

                if (average_wbc_dt is None and past_wbc_time is not None):
                    num_wbc_frames += 1
                    average_wbc_dt = current_wbc_time - past_wbc_time
                elif (past_wbc_time is not None):
                    num_wbc_frames += 1
                    est_wbc_dt = current_wbc_time - past_wbc_time
                    average_wbc_dt = ((average_wbc_dt * (num_wbc_frames - 1)) + est_wbc_dt) / num_wbc_frames
                    
                past_wbc_time = current_wbc_time
        # Check if goal has been reached yet


    planning_time = time_to_reach_goal - time_to_start
    print("[--------------------------------TRIAL OVER--------------------------------]")
    print("Result:                              ", "SUCCESS" if goal_state is not None else "FAILURE")
    print("Planning time:                       ", str(planning_time) + " seconds")
    print("Total path length:                   ", str(path_length) + " meters")
    print("Number of steps:                     ", str(num_steps) + " steps")
    average_torso_speed = path_length / planning_time
    print("Average torso speed:                 ", str(average_torso_speed) + " m/s")
    print("Average elevation map update rate:   ", str(1.0 / average_map_dt) + " Hz")
    print("Average MPC update rate:             ", str(1.0 / average_mpc_dt) + " Hz")
    print("Average WBC update rate:             ", str(1.0 / average_wbc_dt) + " Hz")



def main():
    parse_rosbag()
    

if __name__ == "__main__":
  main()
