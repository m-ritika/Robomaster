#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int16
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
#from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from nav2_msgs.action import NavigateToPose

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        '''self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')'''
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self._amclPoseCallback,
                                                       amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()


    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


class HealthSubscriber(Node):

    def __init__(self):
        super().__init__('health_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            '/health',
            self.get_health_state,
            10)
        self.subscription  # prevent unused variable warning
        
    def get_health_state(self, msg):
    	
    	global low_health
    	low_health = False
    	min_thresh = 95
    	
    	curr = msg
    	prev = curr
       
    	# Check for low health
    	# if prev_health_state >= low_helath_min_threshold and this_health_state <low_health_min_threshold:
    	if curr.data < min_thresh: low_health = True
    	if low_health: print("low health: moving")
    	        
class AmmoSubscriber(Node):

    def __init__(self):
        super().__init__('ammo_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            '/ammo',
            self.get_ammo_state,
            10)
        self.subscription  # prevent unused variable warning
        
    def get_ammo_state(self, msg):

    	
    	global low_ammo
    	low_ammo = False
    	min_thresh = 600
    	
    	curr = msg
    	prev = curr
       
    	# Check for low health
    	# if prev_health_state >= low_helath_min_threshold and this_health_state < low_health_min_threshold:
    	if curr.data < min_thresh: low_ammo = True
    	if low_ammo: print("low ammo: moving")
    	
    	#return low_ammo
    	

class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goalpose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
    	goal_pose = PoseStamped()
    	goal_pose.header.frame_id = 'map'
    	goal_pose.pose.position.x = 10.5
    	goal_pose.pose.position.y = 6.5
    	goal_pose.pose.position.z = 0.1
    	goal_pose.pose.orientation.x = 0.0
    	goal_pose.pose.orientation.y = 0.0
    	goal_pose.pose.orientation.z = 0.0
    	goal_pose.pose.orientation.w = 0.0
    	
    	goal_pose1 = PoseStamped()
    	goal_pose1.header.frame_id = 'map'
    	goal_pose1.pose.position.x = 6.0
    	goal_pose1.pose.position.y = 4.0
    	goal_pose1.pose.position.z = 0.1
    	goal_pose1.pose.orientation.x = 0.0
    	goal_pose1.pose.orientation.y = 0.0
    	goal_pose1.pose.orientation.z = 0.0
    	goal_pose1.pose.orientation.w = 0.0

    	self.get_logger().info('Publishing')
    	if low_health:
    		self.publisher_.publish(goal_pose)
    	elif low_ammo: 
    		self.publisher_.publish(goal_pose1)
    		
    		
class InitPublisher(Node):

    def __init__(self):
        super().__init__('init_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
    	goal_pose = PoseWithCovarianceStamped()
    	goal_pose.header.frame_id = 'map'
    	goal_pose.pose.pose.position.x = 1.5
    	goal_pose.pose.pose.position.y = 1.5
    	goal_pose.pose.pose.position.z = 0.0
    	goal_pose.pose.pose.orientation.x = 0.0
    	goal_pose.pose.pose.orientation.y = 0.0
    	goal_pose.pose.pose.orientation.z = 0.0
    	goal_pose.pose.pose.orientation.w = 0.1
    	
    	
    	self.get_logger().info('Publishing')
    	self.publisher_.publish(goal_pose)


def main(args=None):

    rclpy.init(args=args)
    
    init_publisher = InitPublisher()
    #rclpy.spin(init_publisher)
    rclpy.spin_once(init_publisher)
    init_publisher.destroy_node()
    

    health_subscriber = HealthSubscriber()
    ammo_subscriber = AmmoSubscriber()
    goal_pose_publisher = GoalPosePublisher()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(health_subscriber)
    executor.add_node(ammo_subscriber)
    executor.add_node(goal_pose_publisher)


    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #health_subscriber.destroy_node()
    #ammo_subscriber.destroy_node()
    #odom_subscriber.destroy_node()
    #goal_pose_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
