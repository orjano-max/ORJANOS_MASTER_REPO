#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

from rclpy.node import Node

from std_msgs.msg import String 
import time

def create_pose_stamped( nav , position_x , position_y , orientation_z): 
    q_x , q_y , q_z , q_w = tf_transformations.quaternion_from_euler( 0.0 , 0.0 , orientation_z )
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

class MinimalPublisher(Node):
    
    action_= ""

    def __init__(self):
        super().__init__('husky_master_node')
        self.publisher_ = self.create_publisher(String, 'vx300/action', 10) 
        self.action_publisher()   

    def action_publisher(self):
        # Action can be, "pick", "place", "calib"
        # All other messages will be intergreted as "none" and manipulator goes to sleep
        msg = String()
        msg.data = self.action_
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def set_action(self, action):
        self.action_ = action


def main(args=None):
    rclpy.init(args=args)

    
    minimal_publisher = MinimalPublisher()

    minimal_publisher.set_action("none")
    rclpy.spin_once(minimal_publisher)

    """ nav = BasicNavigator()
    # --- Wait for Nav2 
    nav.waitUntilNav2Active()
    # --- Send Nav2 goal 
    goal_pose = create_pose_stamped(nav , 2.0 , 0.0 , 3.14)
    nav.goToPose(goal_pose) """

    start_time = time.monotonic()
    timeout = 10 # timeout value in seconds

    """ while rclpy.ok() and not reachedPickLoc:

        # Check if navigation task is complete
        if nav.isTaskComplete():
            reachedPickLoc = True

        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached """

    while rclpy.ok():

        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached
        
    
    reachedPickLoc = True ## TESTING TESTING

    if reachedPickLoc:
        minimal_publisher.set_action("pick")
        rclpy.spin_once(minimal_publisher)


    """ # --- Send Nav2 goal 
    goal_pose = create_pose_stamped(nav , 2.0 , 1.0 , 3.14)
    nav.goToPose(goal_pose)

    while rclpy.ok() and not reachedPlaceLoc:

        # Check if navigation task is complete
        if nav.isTaskComplete():
            reachedPlaceLoc = True

        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached """

    while rclpy.ok():

        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached
    
    
    reachedPlaceLoc = True ## TESTING TESTING

    if reachedPlaceLoc:
        minimal_publisher.set_action("place")
        rclpy.spin_once(minimal_publisher)

    """
    # --- Send Nav2 goal 
    goal_pose = create_pose_stamped(nav , 0 , 0 , 0)
    nav.goToPose(goal_pose)

      while rclpy.ok() and not reachedHome:

        # Check if navigation task is complete
        if nav.isTaskComplete():
           reachedHome = True

        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached """

    while rclpy.ok() :
        ## Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            break # Jump out if timeout is reached
    
            

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' : 
    main()