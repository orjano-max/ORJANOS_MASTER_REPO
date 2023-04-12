#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import threading
import time

class HuskyMasterNode(BasicNavigator):
    
    action_= ""
    action_status_ = ""
    __manipulator_model = "" # This variable is set to vx300 by default upon init. 
    object = ""
    dimensions = [0.0, 0.0, 0.0]
    pick_loc = [0.0, 0.0, 0.0]


    def __init__(self, manipulator_model = "vx300"):
        super().__init__()

        if not self.has_parameter("object"):
            self.declare_parameter("object","case")
        if not self.has_parameter("dimensions"):
            self.declare_parameter("dimensions", [0.15, 0.045, 0.04])
        if not self.has_parameter("pick_loc"):
            self.declare_parameter("pick_loc", [0.8, 0.9, -1.51])

        self.object = self.get_parameter("object").get_parameter_value().string_value
        self.dimensions = self.get_parameter("dimensions").get_parameter_value().string_value
        self.pick_loc = self.get_parameter("pick_loc").get_parameter_value().double_array_value

        self.__manipulator_model = manipulator_model

        self.publisher_ = self.create_publisher(
            String, 
            self.__manipulator_model+'/action', 
            10) 
        
        self.subscription = self.create_subscription(
            String,
            self.__manipulator_model+'/action_status',
            self.action_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.mutex = threading.Lock()

    def action_callback(self, msg):
        self.action_status_ = msg.data
        self.get_logger().info('Manipulator status: "%s"' % msg.data)
        
    def set_action(self, action):
        with self.mutex:
            self.action_ = action

    def action_publisher(self):
        # Action can be, "pick", "place", "calib" and "sleep"
        # All other messages will be interpreted as "none" and manipulator wont do anything
        with self.mutex:
            msg = String()
            msg.data = self.action_
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def run(self):
        rclpy.spin(self)

    def create_pose_stamped(self, position_x , position_y , orientation_z): 
        q_x , q_y , q_z , q_w = tf_transformations.quaternion_from_euler( 0.0 , 0.0 , orientation_z )
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

def main(args=None):

    rclpy.init(args=args)

    reachedPickLoc = False
    pickingFinished = False
    reachedPlaceLoc = False
    placingFinished = False
    reachedHome = False
    timeout = 120 # timeout value in seconds

    # Initiate husky master node
    nav = HuskyMasterNode()
    
    # Wait for Nav2 
    nav.waitUntilNav2Active(localizer="waypoint_follower")

    # Get object name
    object = nav.object
    # Get pick_pose
    pick_pose = nav.create_pose_stamped(nav.pick_loc[0], nav.pick_loc[1], nav.pick_loc[2])
    # Get object dimensions
    object_dimensions = nav.dimensions
        
    # Send Nav2 goal 
    nav.goToPose(pick_pose) 

    # Wait for navigation task to finish
    start_time = time.monotonic()
    while rclpy.ok() and not reachedPickLoc:

        # Check if navigation task is complete
        if nav.isTaskComplete():
            reachedPickLoc = True

        # Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            elapsed_time = 0
            break # Jump out if timeout is reached
    
    # Send pick command
    if reachedPickLoc:
        nav.set_action('pick' + ' ' + object + ' ' + object_dimensions)
        nav.action_publisher()        

    # Wait for picking task to finish
    start_time = time.monotonic()
    while rclpy.ok() and not pickingFinished:

        # Spin to get topic subscriptions
        rclpy.spin_once(nav)

        # Check if picking task is complete
        if nav.action_status_ == "picking finished":
            pickingFinished = True
            
        # Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            elapsed_time = 0
            break # Jump out if timeout is reached

    # Go to place location
    if pickingFinished:
        # Send Nav2 goal 
        place_pose = nav.create_pose_stamped( 0.5, -0.4, 1.51)
        nav.goToPose(place_pose)

    # Wait for navigation task to finish
    start_time = time.monotonic()
    while rclpy.ok() and not reachedPlaceLoc:

        # Check if navigation task is complete
        if nav.isTaskComplete():
            reachedPlaceLoc = True

        # Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            elapsed_time = 0
            break # Jump out if timeout is reached
    
    # Send place command
    if reachedPlaceLoc:
        nav.set_action("place")
        nav.action_publisher()

    # Wait for placing task to finish
    start_time = time.monotonic()
    while rclpy.ok() and not placingFinished:

        # Spin to get topic subscriptions
        rclpy.spin_once(nav)

        # Check if placing task is complete
        if nav.action_status_ == "placing finished":
            placingFinished = True
            
        # Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            elapsed_time = 0
            break # Jump out if timeout is reached

    # Send Nav2 goal 
    goal_pose = nav.create_pose_stamped( 0.0, 0.0, 0.0)
    nav.goToPose(goal_pose)

    # Wait for navigation task to finish
    start_time = time.monotonic()
    while rclpy.ok() and not reachedHome:

        # Check if navigation task is complete
        if nav.isTaskComplete():
           reachedHome = True
           nav.get_logger().info('UGV reached home, shutting down')
           
        # Check elapsed time
        elapsed_time = time.monotonic() - start_time
        if elapsed_time > timeout:
            elapsed_time = 0
            break # Jump out if timeout is reached


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' : 
    main()