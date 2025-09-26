import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import tf2_ros

class VehicleVisualizer(Node):
    def __init__(self):
        super().__init__('vehicle_visualizer')
        # Use TRANSIENT_LOCAL QoS for robot_description subscriptions
        self.qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.nearby_sub = self.create_subscription(PoseStamped, '/nearby_vehicle', self.nearby_callback, 10)
        
        # Subscribe to robot_description topics for each car
        self.robot_desc_subs = {}
        for car_id in ['7', '8', '10', '11']:
            self.robot_desc_subs[car_id] = self.create_subscription(
                String, f'/car_{car_id}/robot_description', self.robot_desc_callback, self.qos_profile)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Track seen vehicles for TF broadcasting
        self.seen_vehicles = set()

    def robot_desc_callback(self, msg):
        # Log receipt of robot_description (optional, for debugging)
        self.get_logger().info(f'Received robot_description: {msg.data[:50]}...')

    def publish_vehicle(self, car_id, position, orientation, timestamp):
        if car_id not in self.seen_vehicles:
            self.seen_vehicles.add(car_id)

        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'car_{car_id}/base_link'
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg):
        car_id = '11'
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        timestamp = msg.header.stamp
        self.publish_vehicle(car_id, position, orientation, timestamp)

    def nearby_callback(self, msg):
        car_id = msg.header.frame_id
        if car_id not in ['7', '8', '10']:  # Filter out 11 to prioritize /odom
            return
        position = msg.pose.position
        orientation = msg.pose.orientation
        timestamp = msg.header.stamp
        self.publish_vehicle(car_id, position, orientation, timestamp)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 ######  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom #Run this command in seperate terminal to connect odom to map ###



