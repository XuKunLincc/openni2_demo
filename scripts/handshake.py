#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import numpy as np
from moveit_msgs.msg import PlanningScene, ObjectColor, MoveItErrorCodes
import copy

home = "home"
handshake_pre = False
datanum = 2
error = 0.05
handshaking = "Handshaked"
ready = "Ready"
bodyout ="BodyOut"
trynum = 10


class HandShake:
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('handshake', anonymous=True)
		self.state = ready
		rospy.Subscriber("right_hand_pose_base", geometry_msgs.msg.Pose, self.callback)

		self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
		self.error_x = datanum * [0]
		self.error_y = datanum * [0]
		self.error_z = datanum * [0]
		self.error_index = 0
		self.initMoveit()
		self.addScene()
		self.goHome()
		rospy.spin()

	def initMoveit(self):
		self.robot =  moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("pick_arm_ur5")

		self.group.set_planner_id("RRTConnectkConfigDefault")
		self.group.set_planning_time(0.5)	
	
		#pose = self.group.get_current_pose()
		#rospy.loginfo(pose)

	def addScene(self):
		rospy.sleep(2)
		rospy.loginfo("add scene")
		table_id = 'table'    
		self.scene.remove_world_object(table_id)
		table_size = [2, 2, 0.04]
		table_pose = PoseStamped()
		table_pose.header.frame_id = "base_link"
		table_pose.pose.position.x = 0
		table_pose.pose.position.y = 0
		table_pose.pose.position.z = -0.01
		table_pose.pose.orientation.w = 1.0
		self.scene.add_box(table_id, table_pose, table_size)

		rospy.sleep(1)


	def goHome(self):

		self.group.set_named_target(home)
		self.group.plan()
		self.group.go(wait=True)
		self.state = ready
		'''
		pose = geometry_msgs.msg.Pose()
		pose.orientation.x = 0.254673315982
		pose.orientation.y = 0.247313977335
		pose.orientation.z = -0.933621403612
		pose.orientation.w = 0.0482532222573

		#pose.orientation.x = 0
		#pose.orientation.y = 0
		#pose.orientation.z = 0
		#pose.orientation.w = 1

		pose.position.x = -0.3
		pose.position.y = 0.4
		pose.position.z = 0.294805307495

		rospy.loginfo(pose)
		self.group.set_pose_target(pose)
		self.group.plan()
		self.group.go(wait=True)
		'''

	def handshake(self, pose):
		if self.state == handshaking:
			print("--------------------handshaked-------------")
			return None

		print("--------------- handshaking------------------")
		if handshake_pre:
			self.group.set_named_target("start")
			self.group.plan()
			self.group.go(wait=True)

		pose.orientation.x = 0.254673315982
		pose.orientation.y = 0.247313977335
		pose.orientation.z = -0.933621403612
		pose.orientation.w = 0.0482532222573

		#rospy.loginfo(pose)
		self.group.set_pose_target(pose)
		#plan = self.group.plan()
		count = 0
		while count <= 10:
			rospy.loginfo(count)
			result = self.group.go(wait=True)
			if result == True:
				self.state = handshaking
				break
			else :
				count += 1

		


	def checkError(self):
		mean_x = np.mean(self.error_x)
		mean_y = np.mean(self.error_y)
		mean_z = np.mean(self.error_z)

		if mean_x <= error and mean_y <= error and mean_z <= error:
			return True
		else:
			print("--------------error to long--------------")
			return False

	def bodyOut(self):
			print("--------------- bodyout------------------")
			if self.state == handshaking:
				self.state = bodyout
				self.goHome()

			self.error_x = datanum*[0]
			self.error_y = datanum*[0]
			self.error_z = datanum*[0]
			self.error_index = 0


	def callback(self, data):
		position = data.position

		if not "last_pose" in dir(self):
			self.last_pose = data

		if position.x >= -1.0:
			print("error_index = :", self.error_index)
			self.error_x[self.error_index] = position.x - self.last_pose.position.x
			self.error_y[self.error_index] = position.y - self.last_pose.position.y
			self.error_z[self.error_index] = position.z - self.last_pose.position.z
			self.error_index = self.error_index + 1
			self.last_pose = data
			if self.error_index == datanum:
				self.error_index = 0
				if self.checkError():
					self.handshake(data)
		else:
			self.bodyOut()

if __name__ == '__main__':
	HandShake()
