import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
# from tf_transformations import quaternion_from_euler

import odrive
from odrive.enums import *
import time
import math
from tf2_ros import TransformBroadcaster
import numpy as np


class wheel_controller(Node):
    UPDATE_RATE = 1
    CALIBON = False
    TREAD = 0.550
    WHEEL_DIA = 0.171 
    CIRCUMFERENCE = 2.0 * math.pi * (WHEEL_DIA / 2.0)

    def __init__(self,**args):    
        super().__init__('botwheel_controller_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #ODrive setup start
        print("Connecting to ODrive...")
        self.odrive_left = odrive.find_any(serial_number = "394F35773231")#decimal "63012362203697"
        self.odrive_right = odrive.find_any(serial_number = "393535663231")#decimal "62900691939889"
        print("Connected to ODrive.")
    
        if(self.CALIBON):
            self.get_logger().info("Starting calibration...")
            self.odrive_left.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.odrive_right.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.odrive_left.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            while self.odrive_right.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            self.get_logger().info("Calibration complete.")

        self.odrive_left.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_right.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.position_x = 0.0
        self.position_y = 0.0
        self.angular_z = 0.0
        self.prev_left_enc = self.odrive_left.axis0.pos_estimate
        self.prev_right_enc = self.odrive_right.axis0.pos_estimate
        self.create_timer(1.0 / self.UPDATE_RATE, self.publish_odometry)
        #self.prev_time = time.time()

    def cmd_vel_callback(self,msg):
        linear_x = msg.linear.x
        angular_z = -msg.angular.z
        
        ref_left_wheel_speed = linear_x - (self.TREAD/2) * angular_z
        ref_right_wheel_speed = linear_x + (self.TREAD/2) * angular_z

        # ref_left_wheel_speed_rpm = int((ref_left_wheel_speed/self.CIRCUMFERENCE) * 60)
        # ref_right_wheel_speed_rpm = int((ref_right_wheel_speed/self.CIRCUMFERENCE) * 60)
        # #For debug
        # print("left : %3d[rpm] %lf[m/s]\tright : %3d[rpm] %lf[m/s]" % (ref_left_wheel_speed_rpm, ref_left_wheel_speed, ref_right_wheel_speed_rpm, ref_right_wheel_speed))
        
        self.odrive_left.axis0.controller.input_vel=ref_left_wheel_speed
        self.odrive_right.axis0.controller.input_vel=-ref_right_wheel_speed

    def publish_odometry(self):
        left_distance = (self.odrive_left.axis0.pos_estimate - self.prev_left_enc) * self.CIRCUMFERENCE
        right_distance = (-(self.odrive_right.axis0.pos_estimate - self.prev_right_enc)) * self.CIRCUMFERENCE
        self.prev_left_enc = self.odrive_left.axis0.pos_estimate
        self.prev_right_enc = self.odrive_right.axis0.pos_estimate

        dt = self.UPDATE_RATE
        # current_time = time.time()
        # dt = current_time - self.prev_time
        # self.prev_time = current_time

        v_r = right_distance / dt
        v_l = left_distance / dt
        velocity = (v_r + v_l) / 2.0 
        distance = (right_distance + left_distance) / 2.0
        theta = (v_r - v_l) / self.TREAD 
        
        self.position_x += distance * math.cos(self.angular_z)
        self.position_y += distance * math.sin(self.angular_z)
        self.angular_z += math.radians(theta * dt)

        self.get_logger().info("---------------------------------")
        self.get_logger().info("x :" + str(int(1000*self.position_x)))
        self.get_logger().info("y :" + str(int(1000*self.position_y)))
        self.get_logger().info("theta :" + str(int(np.rad2deg(self.angular_z))))
        self.get_logger().info("---------------------------------")

        quaternion = self.quaternion_from_euler(0, 0, self.angular_z)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'odom'

        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.twist.twist.linear.x = velocity
        odom_msg.twist.twist.angular.z = theta

        self.publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(t)


    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
def main():
    try:
        rclpy.init()
        node = wheel_controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("Releasing ODrive connection...")
    #todo : Add disconnect to odirve funciton
    time.sleep(0.1)
    print("ODrive connection closed.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


 



        

 
