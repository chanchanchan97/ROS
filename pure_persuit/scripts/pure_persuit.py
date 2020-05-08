#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist

from styx_msgs.msg import Lane, Waypoint

from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64

import tf
import rospy

HORIZON = 3.0
L = 0.16
T = 0.16

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)

		rospy.Subscriber('/rear_pose', PoseStamped, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/velocity', TwistStamped, self.vel_cb, queue_size = 1)
		rospy.Subscriber('/final_waypoints', Lane, self.lane_cb, queue_size = 1)

		self.left_vel_pub =rospy.Publisher('/rear_left_velocity_controller/command', Float64, queue_size = 10)
		self.right_vel_pub = rospy.Publisher('/rear_right_velocity_controller/command', Float64, queue_size = 10)
		self.left_steer_pub = rospy.Publisher('/left_bridge_position_controller/command', Float64, queue_size = 10)
		self.right_steer_pub = rospy.Publisher('/right_bridge_position_controller/command', Float64, queue_size = 10)
		
		self.currentPose = None
		self.currentVelocity = None
		self.currentWaypoints = None

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("pure persuit starts")
		while not rospy.is_shutdown():
			if self.currentPose and self.currentVelocity and self.currentWaypoints:
				self.calculateTwistCommand()
			rate.sleep()

	def pose_cb(self,data):
		self.currentPose = data

	def vel_cb(self,data):
		self.currentVelocity = data

	def lane_cb(self,data):
		self.currentWaypoints = data
		
	def ackermann_steering_control(self, velocity, radian):
		global left_angle, left_speed, right_angle, right_speed
		print 'radian = ', radian	
		if radian > 0:
			inside_radius = L / math.tan(radian) - T / 2
			outside_radius = L / math.tan(radian) + T / 2
		else:
			outside_radius = L / math.tan(radian) - T / 2
			inside_radius = L / math.tan(radian) + T / 2
	
		outside_speed = velocity * ( 1 + T * math.tan(abs(radian)) / ( 2 * L ) )
		inside_speed = velocity * ( 1 - T * math.tan(abs(radian)) / ( 2 * L ) )
	
		inside_angle = math.atan( L / inside_radius )
		outside_angle = math.atan( L / outside_radius )
	
		if radian > 0:
			left_angle = outside_angle
			left_speed = outside_speed
			right_angle = inside_angle
			right_speed = inside_speed
		else:
			right_angle = outside_angle
			right_speed = outside_speed
			left_angle = inside_angle
			left_speed = inside_speed
		
		print 'left_speed = ', left_speed, 'right_speed = ', right_speed

	def calculateTwistCommand(self):
		lad = 0.0 #look ahead distance accumulator
		targetIndex = len(self.currentWaypoints.waypoints) - 1
		for i in range(len(self.currentWaypoints.waypoints)):
			if((i+1) < len(self.currentWaypoints.waypoints)):
				this_x = self.currentWaypoints.waypoints[i].pose.pose.position.x
				this_y = self.currentWaypoints.waypoints[i].pose.pose.position.y
				next_x = self.currentWaypoints.waypoints[i+1].pose.pose.position.x
				next_y = self.currentWaypoints.waypoints[i+1].pose.pose.position.y
				lad = lad + math.hypot(next_x - this_x, next_y - this_y)
				if(lad > HORIZON):
					targetIndex = i+1
					break
 
		targetWaypoint = self.currentWaypoints.waypoints[targetIndex]

		targetSpeed = self.currentWaypoints.waypoints[0].twist.twist.linear.x

		targetX = targetWaypoint.pose.pose.position.x
		targetY = targetWaypoint.pose.pose.position.y		
		currentX = self.currentPose.pose.position.x
		currentY = self.currentPose.pose.position.y
		print '[',targetX, targetY, ']'
		#get vehicle yaw angle
		quanternion = (self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y, self.currentPose.pose.orientation.z, self.currentPose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)
		yaw = euler[2]
		#get angle difference
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw
		print 'alpha = ', alpha
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		if(l > 0.5):
			theta = math.atan(2 * L * math.sin(alpha) / l)
			print 'theta = ', theta
			self.ackermann_steering_control(targetSpeed, -theta)
			# #get twist command
			left_steer = Float64()
			right_steer = Float64()
			left_vel = Float64()
			right_vel = Float64()
			left_steer.data = left_angle
			right_steer.data = right_angle
			left_vel.data = left_speed
			right_vel.data = right_speed 
		else:
			left_steer = Float64()
			right_steer = Float64()
			left_vel = Float64()
			right_vel = Float64()
			left_steer.data = 0
			right_steer.data = 0
			left_vel.data = 0
			right_vel.data = 0
			
		self.left_vel_pub.publish(left_vel)
		self.right_vel_pub.publish(right_vel)
		self.left_steer_pub.publish(left_steer)
		self.right_steer_pub.publish(right_steer)

if __name__ == '__main__':
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

