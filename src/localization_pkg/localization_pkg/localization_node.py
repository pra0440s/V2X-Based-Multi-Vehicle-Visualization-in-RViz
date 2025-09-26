import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mocap_msgs.msg import RigidBodies
import numpy as np

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(RigidBodies, '/pose_modelcars', self.pose_callback, 10)
        self.timer = self.create_timer(0.0333, self.publish_odom)  # 30 Hz

        # Kalman Filter Initialization
        self.dt = 0.0333
        self.x = np.array([0, 0, 0, 0, 0], dtype=float)  # [x, y, vx, vy, theta]
        self.P = np.eye(5) * 0.1
        self.F = np.array([[1, 0, self.dt, 0, 0],
                           [0, 1, 0, self.dt, 0],
                           [0, 0, 1, 0, 0],
                           [0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])
        self.Q = np.eye(5) * 0.01
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1]])
        self.R = np.eye(3) * 0.05
        self.pose = np.array([0.0, 0.0, 0.0], dtype=float)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.prev_pose = None

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        z = np.array([measurement[0], measurement[1], measurement[2]])
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(5) - np.dot(K, self.H)), self.P)

    def pose_callback(self, msg):
        for rigidbody in msg.rigidbodies:
            if rigidbody.rigid_body_name == '5':
                new_pose = np.array([rigidbody.pose.position.x, rigidbody.pose.position.y])
                new_theta = np.arctan2(2.0 * (rigidbody.pose.orientation.w * rigidbody.pose.orientation.z),
                                       1.0 - 2.0 * (rigidbody.pose.orientation.z**2))
                if self.prev_pose is not None:
                    measurement = np.array([new_pose[0], new_pose[1], new_theta])
                    self.predict()
                    self.update(measurement)
                self.pose = np.array([new_pose[0], new_pose[1], 0.0])
                self.quat = np.array([rigidbody.pose.orientation.x, rigidbody.pose.orientation.y,
                                      rigidbody.pose.orientation.z, rigidbody.pose.orientation.w])
                self.prev_pose = new_pose.copy()
                self.get_logger().info(f"Filtered Pose for ID 05: x={self.x[0]:.2f}, y={self.x[1]:.2f}")

    def publish_odom(self):
        timestamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = "map"
        msg.child_frame_id = "car_05/base_link"
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = float(self.pose[2])
        msg.pose.pose.orientation.x = float(self.quat[0])
        msg.pose.pose.orientation.y = float(self.quat[1])
        msg.pose.pose.orientation.z = float(self.quat[2])
        msg.pose.pose.orientation.w = float(self.quat[3])
        msg.twist.twist.linear.x = float(self.x[2])
        msg.twist.twist.linear.y = float(self.x[3])
        msg.twist.twist.angular.z = float(self.x[4])
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
