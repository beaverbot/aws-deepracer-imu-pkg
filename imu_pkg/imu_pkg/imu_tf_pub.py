#!/usr/bin/env python
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Temperature, Imu
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class ImuTFPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_pub')
        
        self.sub_topic = self.declare_parameter("~imu_sub_topic", "/imu/data").value
        self.parent_link = self.declare_parameter("~imu_parent_link", "base_link").value
        self.child_link = self.declare_parameter("~imu_link", "imu_link").value
        imu_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        # imu_qos = rclpy.qos.QoSPresetProfiles.get_from_short_key('sensor_data')
        #self.create_timer(1.0/self.rate, self.read_imu_and_publish_sensormsg)  
        self.create_subscription(Imu, self.sub_topic, self.imu_data_callback, imu_qos)
        self.get_logger().info(f"imu_tf_pub:: started ... sub_topic::{self.sub_topic} parent_link::{self.parent_link} child_link::{self.child_link}")
    
    def imu_data_callback(self, msg):
        #self.get_logger().info("received imu/data msg")
        br = tf2_ros.TransformBroadcaster(self)
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_link
        t.child_frame_id = self.child_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        br.sendTransform(t)


    
    
   
def main(args=None):
    

    rclpy.init(args=args)
    try:
        imu_tf_pub = ImuTFPublisher()
        rclpy.spin(imu_tf_pub)
    except rclpy.exceptions.ROSInterruptException:
        pass

    imu_tf_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()