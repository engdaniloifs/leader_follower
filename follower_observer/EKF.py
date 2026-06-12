#! /usr/bin/env python3


# Generic python packages
import time
import numpy as np

# ROS specific packages
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose,Twist, TwistStamped # Pose with ref frame and timestamp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

class EKF(Node):

    def __init__(self):
        super().__init__('EKF_node')

        # =========================================================
        # Parameters
        # =========================================================
        self.declare_parameter('qcarnumber', 1)
        self.qcarnumber = self.get_parameter('qcarnumber').get_parameter_value().integer_value
    
        self.dt = 0.05  # time step (s)

        self.max_steering_angle = 0.6  # steering limit [rad]
        self.steering_angle = 0.0       # current steering angle
        self.ell = 0.256                # wheelbase

        # =========================================================
        # Publisher(s)
        # =========================================================
        self.publisher = self.create_publisher(Twist, 'cmd_vel_nav', 1)
        self.solve_time_publisher = self.create_publisher(Float32, 'controller_solve_time', 1)
        # =========================================================
        # Subscriptions (main loop + inputs)
        # =========================================================
        self.estimate_state_timer = self.create_timer(self.dt, self.estimate_state)

        self.subscription_vycon = self.create_subscription(
            PoseStamped,
            'vrpn_pose',
            self.pose_vycon_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )


        self.subscription_stop_flag = self.create_subscription(
            Bool,
            '/qcar/stop',
            self.stop_experiment_callback,
            10
        )

        self.path_following_enable_sub = self.create_subscription(
            Bool,
            'path_following_enable',
            self.path_following_enable_callback,
            10
        )

        self.get_logger().info("EKF follower node has been started.")


    # mocap pose callback
    def pose_vycon_callback(self,msg):
      self.position = msg.pose.position
      orientation = msg.pose.orientation  
      rotation = [orientation.x, orientation.y, orientation.z, orientation.w]
      roll, pitch, self.yaw = Rotation.from_quat(rotation).as_euler('xyz', degrees=False)
    


    def stop_experiment_callback(self, msg: Bool):
      self.FSM = msg.data
      if not self.FSM:
        self.get_logger().info("User called STOP ")
        self.nav_command(0.0,self.steering_angle)
      else:
        self.get_logger().info("User called START ")
    

    def control_algorithm(self,msg):
        time_start = time.time()
        # current position

        x = np.array([self.position.x,self.position.y, self.yaw])
        if self.FSM == 1:

          commands_star = self.controller.solve(x)
      
          speed_command = commands_star[0]
          phi = commands_star[1]
    
          self.steering_angle = np.clip(phi, -self.max_steering_angle, self.max_steering_angle)
        else:
             speed_command = 0.0
      
        
        if x[0] > 3.4 or x[0] <-3.4 or x[1] >2 or x[1] <-2:
          speed_command = 0.0
        time_end = time.time()
        elapsed_time = time_end - time_start
        self.nav_command(speed_command,self.steering_angle)
        self.publish_solve_time(elapsed_time)
           

    def nav_command(self,speed_command, steering_angle):
      QCarCommands = Twist()
      QCarCommands.linear.x = speed_command
      QCarCommands.angular.z = steering_angle
      self.publisher.publish(QCarCommands)

    def publish_solve_time(self, solve_time):
      solve_time_msg = Float32()
      solve_time_msg.data = solve_time
      self.solve_time_publisher.publish(solve_time_msg)
    
      

       


def main():

  # Start the ROS 2 Python Client Library
  rclpy.init()

  node = EKF()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
        pass
      
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()