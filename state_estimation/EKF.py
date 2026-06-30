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
    
        self.dt = 0.02  # time step (s)

        self.max_steering_angle = 0.6  # steering limit [rad]
        self.steering_angle = 0.0       # current steering angle
        self.ell = 0.256                # wheelbase
        self.tau_phi = 0.16              # steering time constant

        self.measured_omega = 0.0
        self.measured_v_x = 0.0
        self.measured_v_y = 0.0
        self.x_measured = 0.0
        self.y_measured = 0.0
        self.measured_yaw = 0.0
        self.command_steering_angle = 0.0
        self.steering_angle_rate = 0.0

        self.prev_measured_v_x = 0.0
        self.prev_measured_v_y = 0.0
        self.prev_measured_omega = 0.0
        self.prev_command_steering_angle = 0.0

        # self.x_hat = 0.0
        # self.y_hat = 0.0
        # self.yaw_hat = 0.0
        # self.v_hat = 0.0
        # self.omega_hat = 0.0
        # self.steering_angle_hat = 0.0
        # self.steering_rate_hat = 0.0
        self.states_hat = np.zeros(4)  # [x, y, yaw, steering_angle]

        self.P = np.diag([
                            6.0**2,                    # x
                            4.0**2,                    # y
                            np.deg2rad(90.0)**2,       # yaw
                            np.deg2rad(3.0)**2         # steering angle
                        ])
        self.Q = np.diag([
                          0.05**2,                  # x model noise: 1 cm per step
                          0.05**2,                  # y model noise: 1 cm per step
                          np.deg2rad(1.0)**2,        # yaw model noise
                          np.deg2rad(5.0)**2         # steering model noise
                      ])

        # Measurement noise covariance
        self.R = np.diag([
            0.001**2,                  # x measurement noise: 1 cm
            0.001**2,                  # y measurement noise: 1 cm
            np.deg2rad(0.1)**2,        # yaw measurement noise
            np.deg2rad(5.0)**2         # omega measurement noise, rad/s
        ])
        

        # =========================================================
        # Publisher(s)
        # =========================================================
        self.publisher_estimated_states = self.create_publisher(Odometry, 'estimated_states', 1)
        self.solve_time_publisher = self.create_publisher(Float32, 'filter_solver_time', 1)



        # =========================================================
        # Subscriptions (main loop + inputs)
        # =========================================================
        self.estimate_state_timer = self.create_timer(self.dt, self.estimate_state)

        self.subscription_pose_vycon = self.create_subscription(
            PoseStamped,
            '/qcar2_2/vrpn_mocap/Qcar2_2/pose',
            self.pose_vycon_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )

        self.subscription_velocity_vycon = self.create_subscription(
          TwistStamped,
          '/qcar2_2/vrpn_mocap/Qcar2_2/twist',
          self.velocity_vycon_callback,
          QoSProfile(
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    depth=10
                                ))
        
        self.subscription_steering_angle = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.steering_angle_callback, 10
        )



        # self.subscription_stop_flag = self.create_subscription(
        #     Bool,
        #     '/qcar/stop',
        #     self.stop_experiment_callback,
        #     10
        # )


        self.get_logger().info("EKF follower node has been started.")


    # mocap pose callback
    def pose_vycon_callback(self,msg):
      self.measured_position = msg.pose.position
      self.x_measured = self.measured_position.x
      self.y_measured = self.measured_position.y
      orientation = msg.pose.orientation  
      rotation = [orientation.x, orientation.y, orientation.z, orientation.w]
      roll, pitch, self.measured_yaw = Rotation.from_quat(rotation).as_euler('xyz', degrees=False)

    def velocity_vycon_callback(self, msg):
      self.measured_v_x = msg.twist.linear.x
      self.measured_v_y = msg.twist.linear.y
      self.measured_omega = msg.twist.angular.z

    def steering_angle_callback(self, msg):
      self.command_steering_angle = msg.angular.z


    # def stop_experiment_callback(self, msg: Bool):
    #   self.FSM = msg.data
    #   if not self.FSM:
    #     self.get_logger().info("User called STOP ")
    #     self.nav_command(0.0,self.steering_angle)
    #   else:
    #     self.get_logger().info("User called START ")

    def estimate_state(self):
      # Check if we have received the necessary data to perform state estimation
      time_start = time.time()

      state_prev = self.states_hat.copy()

      self.predict_state()

      self.predict_covariance(state_prev)

      self.update_with_measurements()

      self.save_prev_estimation_variables()

      self.publish_estimated_state()
      
      time_end = time.time()
      solve_time = time_end - time_start

      self.publish_solve_time(solve_time)
      
    def predict_state(self):
      # Unpack the current state estimates
      x, y, yaw, steering_angle = self.states_hat

      speed = (
                self.prev_measured_v_x * np.cos(yaw)
                + self.prev_measured_v_y * np.sin(yaw)
              )

      x_dot = speed * np.cos(yaw)
      y_dot = speed * np.sin(yaw)
      yaw_dot = speed / self.ell * np.tan(steering_angle)
  

      self.steering_angle_rate = (self.prev_command_steering_angle - steering_angle) / self.tau_phi

      self.states_hat[0] += x_dot * self.dt
      self.states_hat[1] += y_dot * self.dt
      self.states_hat[2] = self.wrap_angle(yaw + yaw_dot * self.dt)
      self.states_hat[3] += self.steering_angle_rate * self.dt

    def predict_covariance(self, states_prev):
      x, y, yaw, steering_angle = states_prev

      dt = self.dt
      L = self.ell
      tau_phi = self.tau_phi

      speed = (
          self.prev_measured_v_x * np.cos(yaw)
          + self.prev_measured_v_y * np.sin(yaw)
            )

      sec_phi_squared = 1.0 / (np.cos(steering_angle) ** 2)

      F = np.array([
          [1.0, 0.0, -dt * speed * np.sin(yaw), 0.0],
          [0.0, 1.0,  dt * speed * np.cos(yaw), 0.0],
          [0.0, 0.0, 1.0, dt * speed / L * sec_phi_squared],
          [0.0, 0.0, 0.0, 1.0 - dt / tau_phi]])

      self.P = F @ self.P @ F.T + self.Q

    def update_with_measurements(self):
      # Measurement vector
      
      z = np.array([self.x_measured, self.y_measured, self.measured_yaw, self.measured_omega])

      # Measurement prediction
      x, y, yaw, steering_angle = self.states_hat  # Direct measurement of the state
      
      speed = (
          self.measured_v_x * np.cos(yaw)
          + self.measured_v_y * np.sin(yaw)
      )

      h = np.array([
                    x,
                    y,
                    yaw,
                    speed / self.ell * np.tan(steering_angle)
                ])
                      
      
      sec_phi_squared = 1.0 / (np.cos(steering_angle) ** 2)
      

      # Measurement matrix
      H = np.array([
          [1.0, 0.0, 0.0, 0.0],
          [0.0, 1.0, 0.0, 0.0],
          [0.0, 0.0, 1.0, 0.0],
          [0.0, 0.0, 0.0, speed / self.ell * sec_phi_squared]
      ])

      y = z - h  # Measurement residual
      y[2] = self.wrap_angle(y[2])  # Wrap yaw residual

      S = H @ self.P @ H.T + self.R  # Residual covariance
      K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain

      self.states_hat += K @ y  # Update state estimate
      self.states_hat[2] = self.wrap_angle(self.states_hat[2])  # Wrap yaw estimate
      self.steering_angle_rate = (
                                  self.command_steering_angle - self.states_hat[3]
                              ) / self.tau_phi

      I = np.eye(4)
      self.P = (I - K @ H) @ self.P  # Update covariance

    def publish_estimated_state(self):
      # Publish the estimated state as a PoseStamped message
      msg = Odometry()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = 'map'  # or 'odom' depending on your setup

      msg.pose.pose.position.x = self.states_hat[0]
      msg.pose.pose.position.y = self.states_hat[1]
      msg.pose.pose.position.z = 0.0

      yaw = self.states_hat[2]
      quat = Rotation.from_euler('z', yaw).as_quat()
      msg.pose.pose.orientation.x = quat[0]
      msg.pose.pose.orientation.y = quat[1]
      msg.pose.pose.orientation.z = quat[2]
      msg.pose.pose.orientation.w = quat[3]

      speed = (
          self.measured_v_x * np.cos(yaw)
          + self.measured_v_y * np.sin(yaw)
      )

      msg.twist.twist.linear.x = speed
      msg.twist.twist.linear.y = 0.0
      msg.twist.twist.linear.z = 0.0
      msg.twist.twist.angular.x = self.states_hat[3]  # steering angle
      msg.twist.twist.angular.y = self.steering_angle_rate  # steering angle rate
      msg.twist.twist.angular.z = self.measured_omega

      self.publisher_estimated_states.publish(msg)

    def save_prev_estimation_variables(self):
      self.prev_command_steering_angle = self.command_steering_angle
      self.prev_measured_v_x = self.measured_v_x
      self.prev_measured_v_y = self.measured_v_y
      self.prev_measured_omega = self.measured_omega

    def publish_solve_time(self, solve_time):
      solve_time_msg = Float32()
      solve_time_msg.data = solve_time
      self.solve_time_publisher.publish(solve_time_msg)


    def wrap_angle(self, angle):
      # Wrap angle to [-pi, pi]
      return (angle + np.pi) % (2 * np.pi) - np.pi
       
           
    
      

       


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