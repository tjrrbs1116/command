#! /usr/bin/env python
import rclpy
from rclpy.node import Node
import math
import numpy as np
from math import sin, cos
from rclpy.qos import QoSProfile
from piot_can_msgs.msg import CtrlCmd, CtrlFb
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster



class commad3(Node):


    def __init__(self):
      super().__init__('command3')
      qos = QoSProfile(depth=10)
      self.odom_sub = self.create_subscription(Odometry, '/wheel/odometry',self.callback1 , qos)
      self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',qos)
      self.flag = 1


    def callback1(self,msg ) :
      cmd_vel = Twist()
      if msg.pose.pose.position.x <= 5.0 and self.flag ==1:
        cmd_vel.linear.x = 0.8

      elif msg.pose.pose.position.x >= 5.0 and self.flag ==1 :
        self.flag =2
        cmd_vel.linear.x = 0.8

      elif msg.pose.pose.position.x >=0.0 and self.flag ==2 :
        cmd_vel.linear.x = -0.8

      elif msg.pose.pose.position.x <= 0.0 and self.flag ==2:
        self.flag = 1


      self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
	rclpy.init(args=args)
	command_3 = commad3()
	rclpy.spin(command_3)
	command_3.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
