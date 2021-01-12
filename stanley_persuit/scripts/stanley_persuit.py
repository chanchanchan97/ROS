#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist

from styx_msgs.msg import Lane, Waypoint

from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64

from nav_msgs.msg import Path

import tf
import rospy

HORIZON = 0.05
k = 0.2
L =0.16
T = 0.16
final_targetX = 0
final_targetY = 4
flag = 0
flag_alpha = 0

class Stanley:
	def __init__(self):
		rospy.init_node('stanley', log_level=rospy.DEBUG)

		rospy.Subscriber('/front_pose', PoseStamped, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/velocity', TwistStamped, self.vel_cb, queue_size = 1)
		rospy.Subscriber('/final_waypoints', Lane, self.lane_cb, queue_size = 1)

		self.left_vel_pub =rospy.Publisher('/rear_left_velocity_controller/command', Float64, queue_size = 10)
		self.right_vel_pub = rospy.Publisher('/rear_right_velocity_controller/command', Float64, queue_size = 10)
		self.left_steer_pub = rospy.Publisher('/left_bridge_position_controller/command', Float64, queue_size = 10)
		self.right_steer_pub = rospy.Publisher('/right_bridge_position_controller/command', Float64, queue_size = 10)
		#self.real_waypoints_pub = rospy.Publisher('/real_waypoints', Lane, queue_size=1)
		#self.real_path_pub = rospy.Publisher('/real_path', Path, queue_size=1)

		self.currentPose = None
		self.currentVelocity = None
		self.currentWaypoints = None
		
		CurrentYaw = 0
		TargetYaw = 0

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("stanley starts")
		while not rospy.is_shutdown():
			if self.currentPose and self.currentVelocity and self.currentWaypoints:
				left_vel, right_vel, left_steer, right_steer = self.calculateTwistCommand()
				self.publish_waypoints(left_vel, right_vel, left_steer, right_steer)
			rate.sleep()

	def pose_cb(self,data):
		self.currentPose = data

	def vel_cb(self,data):
		self.currentVelocity = data

	def lane_cb(self,data):
		self.currentWaypoints = data
		
	def ackermann_steering_control(self, velocity, radian):
		global left_angle, left_speed, right_angle, right_speed
		
		outside_speed = velocity * ( 1 + T * math.tan(abs(radian)) / ( 2 * L ) )
		inside_speed = velocity * ( 1 - T * math.tan(abs(radian)) / ( 2 * L ) )
		
		#print 'radian = ', radian	
		if radian != 0:
			if radian > 0:
				inside_radius = L / math.tan(radian) - T / 2
				outside_radius = L / math.tan(radian) + T / 2
			else:
				outside_radius = L / math.tan(radian) - T / 2
				inside_radius = L / math.tan(radian) + T / 2
				
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
		else:
			left_angle = 0
			left_speed = velocity
			right_angle = 0
			right_speed = velocity
		
		#print 'left_speed = ', left_speed, 'right_speed = ', right_speed

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
		#print '[',targetX, targetY, ']'
		#print targetWaypoint.pose.pose.orientation
		print self.currentVelocity

		#get waypoint yaw angle
		waypoint_quanternion = (targetWaypoint.pose.pose.orientation.x, targetWaypoint.pose.pose.orientation.y, targetWaypoint.pose.pose.orientation.z, targetWaypoint.pose.pose.orientation.w)
		waypoint_euler = tf.transformations.euler_from_quaternion(waypoint_quanternion)
		TargetYaw = waypoint_euler[2]
		print 'TargetYaw = ', TargetYaw
		
		#get vehicle yaw angle
		quanternion = (self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y, self.currentPose.pose.orientation.z, self.currentPose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)
		CurrentYaw = euler[2]
		print 'CurrentYaw = ', CurrentYaw
		
		#get the difference between yaw angle of vehicle and tangent line of waypoint
		if TargetYaw >= 0:
			alpha = TargetYaw - CurrentYaw
			flag_alpha = 1
		else:
			if TargetYaw * CurrentYaw < 0:
				if (TargetYaw < 0) and (CurrentYaw > 0):
					alpha = (math.pi - CurrentYaw) + (math.pi + TargetYaw)
					flag_alpha = 2
				else:
					alpha = -((math.pi + CurrentYaw) + (math.pi - TargetYaw))
					flag_alpha = 3
			else:
				alpha = TargetYaw - CurrentYaw
				flag_alpha = 4

		print 'flag_alpha = ', flag_alpha
		print '(', currentX, currentY, ')'
		print '(', targetX, targetY, ')'
		print 'alpha = ', alpha
		print 'x = ', targetX - currentX
		print 'y = ', targetY - currentY
		
		#get the error between target position and current position
		if alpha >= 0:
			error = abs( math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2)) )
		else:
			error = -abs( math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2)) )
		print 'error = ', error
		
		#get the velocity of vehicle
		vel = math.sqrt(math.pow(self.currentVelocity.twist.linear.x, 2) + math.pow(self.currentVelocity.twist.linear.y, 2))
		print 'vel = ', vel
		
		#get the nonlinear proportional function from geometry of Vehicular mobility model
		if vel < 0.001:
			delta = 0
		else:
			delta = math.atan(k * error / vel)
		print 'delta = ', delta
		
		#get the distance between final position and current position
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		l2 = math.sqrt(math.pow(currentX - final_targetX, 2) + math.pow(currentY - final_targetY, 2))
		
		if l > 0.001:
			if l2 > 0.1:
				theta = alpha + delta
				print 'theta = ', theta
				#self.ackermann_steering_control(targetSpeed, theta)
				self.ackermann_steering_control(15, -theta)
				#get twist command
				left_steer = Float64()
				right_steer = Float64()
				left_vel = Float64()
				right_vel = Float64()
				left_steer.data = left_angle
				right_steer.data = right_angle
				left_vel.data = left_speed
				right_vel.data = right_speed
				flag = 1
			else:
				left_steer = Float64()
				right_steer = Float64()
				left_vel = Float64()
				right_vel = Float64()
				left_steer.data = 0
				right_steer.data = 0
				left_vel.data = 0
				right_vel.data = 0
				flag = 2
		else:
			left_steer = Float64()
			right_steer = Float64()
			left_vel = Float64()
			right_vel = Float64()
			left_steer.data = 0
			right_steer.data = 0
			left_vel.data = 0
			right_vel.data = 0
			flag = 3

		print 'flag = ', flag
		
		return left_vel, right_vel, left_steer, right_steer
		
	def publish_waypoints(self, left_vel, right_vel, left_steer, right_steer):
		self.left_vel_pub.publish(left_vel)
		self.right_vel_pub.publish(right_vel)
		self.left_steer_pub.publish(left_steer)
		self.right_steer_pub.publish(right_steer)

if __name__ == '__main__':
    try:
        Stanley()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

