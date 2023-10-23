import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x_estimated = np.zeros((3, 1))  # Initialize state estimate (x, y, theta)
        self.P = np.eye(3)  # Initialize state covariance matrix
        
        # Process noise covariance matrix Q
        self.Q = np.diag([0.01, 0.01, 0.01])
        
        # Measurement noise covariance matrix R
        self.R = np.diag([0.1, 0.1, 0.01])

        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub = self.create_publisher(Odometry,
                                                   "/odom_estimated", 1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        z = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [msg.twist.twist.angular.z]])
    
        # Prediction step
        # Predict state estimate
        x_predicted = self.x_estimated
        # Predict state covariance
        P_predicted = self.P + self.Q
        
        # Update step
        # Calculate Kalman Gain
        K = P_predicted @ inv(P_predicted + self.R)
        # Update state estimate
        self.x_estimated = x_predicted + K @ (z - x_predicted)
        # Update state covariance
        self.P = (np.eye(3) - K) @ P_predicted
        
        # Publish the estimated reading
        self.publish_estimated()
    
    def publish_estimated(self):
        # Create Odometry message for estimated reading
        estimated_msg = Odometry()
        
        estimated_msg.header.stamp = self.get_clock().now().to_msg()
        estimated_msg.pose.pose.position.x = self.x_estimated[0, 0]
        estimated_msg.pose.pose.position.y = self.x_estimated[1, 0]
        estimated_msg.twist.twist.angular.z = self.x_estimated[2, 0]
        
        # Publish the estimated reading
        self.estimated_pub.publish(estimated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
