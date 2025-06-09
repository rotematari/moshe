from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
import threading


import time
import numpy as np
import matplotlib.pyplot as plt

def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def get_current_position(basecyclic : BaseCyclicClient):
    """Get the current position of the robot's base."""
    
    feedback = basecyclic.RefreshFeedback()
    position_x = feedback.base.tool_pose_x
    position_y = feedback.base.tool_pose_y
    position_z = feedback.base.tool_pose_z
    return (position_x, position_y, position_z)

def gen_circular_path(current_pose, num_points=100):
    """Generate a circular path in 2D space."""
    
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = np.sin(angles) + current_pose[0]
    y = np.cos(angles) + current_pose[2]
    
    return list(zip(x, y))



def follow_trajectory(base, velocities,speed=0.1,sleep_time=0.01):
    """Follow a path defined by a list of points."""
    for vel in velocities:
        # Create a TwistCommand message
        twist = Base_pb2.TwistCommand()
        twist.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        twist.twist.linear_x = 0.0
        twist.twist.linear_y = vel[0]
        twist.twist.linear_z = vel[1]

        base.SendTwistCommand(twist)
        time.sleep(sleep_time)
    
    twist.twist.linear_x = vel[0]
    twist.twist.linear_y = vel[1]
    twist.twist.linear_z = 0.0
    
    base.SendTwistCommand(twist)
    
    
    
        
def calculate_twist_trajectory(path):
    
    vectors = path[1:] - path[:-1]
    dists = np.linalg.norm(vectors, axis=1)[:, np.newaxis]  # Add new axis for broadcasting
    directions = vectors / dists

    return directions

def GoToStartPose(base):
    """
    Move the arm to the start pose
    
    Parameters
    ----------
    base : BaseClient
        The base client to use to move the arm
    """
    print("Starting action movement ...")
    TIMEOUT_DURATION = 10.0  # seconds
    action = Base_pb2.Action()
    
    # Set the start pose
    action.reach_pose.target_pose.x = 0.6
    action.reach_pose.target_pose.y = 0.0
    action.reach_pose.target_pose.z = 0.4
    action.reach_pose.target_pose.theta_x = 90.0
    action.reach_pose.target_pose.theta_y = 0.0
    action.reach_pose.target_pose.theta_z = 90.0

    Base_pb2.ActionNotification()
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)
    
    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Action movement completed")
    else:
        print("Timeout on action notification wait")
        
        

def plot_path(path):
    """Plot the path in 2D."""
    plt.figure(figsize=(8, 6))
    x, y = zip(*path)
    plt.plot(x, y, marker='o')
    # Paint the first point in orange
    plt.plot(x[0], y[0], marker='o', color='orange', markersize=10, label='Start Point')
    plt.legend()
    x, y = zip(*path)
    plt.plot(x[1:], y[1:], marker='o',color='blue', markersize=5, label='Path Points')
    plt.title("Circular Path")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.axis('equal')
    plt.grid()
    plt.show()


def main():

    
    
    SPEED = 0.10  # m/s
    import utilities
    
    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        
        GoToStartPose(base)
        
        if input("Do you want to follow a path? (y/n): ").strip().lower() != 'y':
            print("Exiting without following path.")
            return
        
        current_pose = get_current_position(base_cyclic)
        print(f"Current position: {current_pose}")
        
        path = gen_circular_path(current_pose, num_points=100)

        directions = calculate_twist_trajectory(np.array(path))
        
        velocities = directions * SPEED
        # plot_path(path)
        print("Following path...")
        follow_trajectory(base, velocities, speed=SPEED, sleep_time=0.01)   
        print("finished sending twist commands")

if __name__ == "__main__":
    main()