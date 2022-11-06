#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from cgi import print_directory
from ctypes.wintypes import PCHAR
from tkinter import N
#from msilib.schema import SelfReg
from typing_extensions import Self
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import tf
import roslib
from time import process_time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
#		self.drone_position = [self.x, self.y, self.z]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint_list=[[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[0,0,23]]

		#self.setpoint = [2,2,23] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

	
		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [16.8,16.8,55.96]#[26,26,45] #0.06 21,(45) ((65))
		self.Ki = [0,0,0] #0.001
		self.Kd = [347.1,347.1,1153.1]#[470,470,950] #0.3 380,(830)((970))

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error = [0,0,0]
		self.prev_error = [0,0,0]
		self.sum_error = [0,0,0]
		self.delta_error = [0,0,0]

		#self.minthrottle = 1000
		#self.maxthrottle = 2000 

		


		self.n=0




		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		
		# #		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------
        

		self.max_values = [1800,1800,1800]
		self.min_values = [1200,1200,1200]
		
		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.050 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	 #This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06# This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0005
		self.Kd[2] = alt.Kd 
		#print(self.Kd[2])

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06
		self.Ki[1] = pitch.Ki * 0.001
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki *0.001
		self.Kd[0] = roll.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	
	#

	def func_wrapper(func):
		
		def inner_func(self):
			waypoints=[[0,0,23],[0,0,23],[0,0,23],[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[2,0,23],[2,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23]]
			#n global 

				
			# print(self.n)

			coord=waypoints[self.n]
			


			func(self,coord=coord)
			
			if self.n < len(waypoints)-1:
				self.n+=1
			else:
				self.n = len(waypoints)-1

			# print(self.n)

		 
		return inner_func

	# def trajectory(self):
	# 	waypoints=[[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[0,0,23]]
	# 	x_setpoint = coord[0]
	# 	y_setpoint = coord[1]
	# 	z_setpoint = coord[2]

	# 	# #print(coord)				#reduce error margin 

	# 	x_max = x_setpoint + 0.1
	# 	y_max = y_setpoint + 0.1
	# 	z_max = z_setpoint + 0.1

	# 	x_min = x_setpoint - 0.1
	# 	y_min = y_setpoint - 0.1
	# 	z_min = z_setpoint - 0.1

	# 	if (x_max < self.drone_position[0] or self.drone_position[0] < x_min) or (y_max < self.drone_position[1] or y_min > self.drone_position[1]) or (z_max < self.drone_position[2] or z_min > self.drone_position[2]):
	# 		func(self,coord)

		

		

# def timer_callback(event):
#     rospy.loginfo("In timer callback")

# rospy.Timer(rospy.Duration(1), timer_callback)
# rospy.spin() # don't forget to spin or else your node will exit












	def pid(self,coord):

		

		x_setpoint = coord[0]
		y_setpoint = coord[1]
		z_setpoint = coord[2]

		# #print(coord)				#reduce error margin 

		x_max = x_setpoint + 0.1
		y_max = y_setpoint + 0.1
		z_max = z_setpoint + 0.1

		x_min = x_setpoint - 0.1
		y_min = y_setpoint - 0.1
		z_min = z_setpoint - 0.1

		# #print(z_max,z_min)

		# # self.error[2] = -(coord[2] - self.drone_position[2])
		# # self.error[0] =  (coord[0] - self.drone_position[0])
		# # self.error[1] =  -(coord[1] - self.drone_position[1])
		# # self.delta_error[2] = self.error[2] - self.prev_error[2]
		# # self.delta_error[1] = self.error[1] - self.prev_error[1]
		# # self.delta_error[0] = self.error[0] - self.prev_error[0]

		# # print(self.error[2] )
		# # print(self.error[1])
		# # print(self.error[0])

		# #self.command_pub.publish(self.cmd)
		# #self.throttle_error_pub.publish(self.error[2])
		# #self.pitch_error_pub.publish(self.error[1])
		# #self.roll_error_pub.publish(self.error[0])

		# #print(self.drone_position)

		# #print(e_drone.drone_position)

		while (x_max < self.drone_position[0] or self.drone_position[0] < x_min) or (y_max < self.drone_position[1] or y_min > self.drone_position[1]) or (z_max < self.drone_position[2] or z_min > self.drone_position[2]):
			

			#t1_start = process_time()
			# print("next")

			self.error[2] = -(coord[2] - self.drone_position[2])
			self.error[0] = (coord[0] - self.drone_position[0])
			self.error[1] = -(coord[1] - self.drone_position[1])
			self.delta_error[2] = self.error[2] - self.prev_error[2]

			# print(self.error[2])
			# print(self.prev_error[2])

			# print(self.delta_error[2])
			self.delta_error[1] = self.error[1] - self.prev_error[1]
			self.delta_error[0] = self.error[0] - self.prev_error[0]

			#print(coord)
			# print(z_max, z_min)
			#print(self.error[2])
			#print(self.error[1])
			#print(self.error[0])

			#print( x,y,z)
			
			#print('inside')
			#print(self.error)

			P2 = self.Kp[2] * self.error[2]
			F = 0
			#print(P2)
			#if self.error[2]<3 and self.error[2]>-3:
			#I2 = self.Ki[2] * self.sum_error[2]
			# if self.sum_error[2]>10:
			# 	self.sum_error[2] = 10
			
			# if self.sum_error[2]<-10:
			# 	self.sum_error[2] = -10
			
			print(self.sum_error[2])

			if self.error[2] < 0.6 and self.error[2] > 0.6 :#and self.sum_error[2]<10 and self.sum_error[2] > -15:
			
				I2 = self.Ki[2] * self.sum_error[2]
			else:
				I2 = 0

			#print(I2)
			#	F= I2
			#else: I2 = F

			D2 = self.Kd[2] * self.delta_error[2]

			# print(self.Kd[2])
			print(self.sum_error[2])
			print(I2)

			self.cmd.rcThrottle = int(1500 + P2 + I2 + D2)  # (P2 + I2 + D2)
			#print(self.cmd.rcThrottle)

			if self.cmd.rcThrottle > 1800:
					self.cmd.rcThrottle = self.max_values[2]
			if self.cmd.rcThrottle < 1200:
					self.cmd.rcThrottle = self.min_values[2]

				#self.delta_error[2]= self.error[2]-self.delta_error[2]
			self.prev_error[2] = self.error[2]
			
			self.sum_error[2] += self.error[2]

			#	self.throttle_sum_error += self.throttle_error
			#	self.throttle_prev_error = self.throttle_error - self.throttle_prev_error
			#	P2 = self.Kp[2]* self.throttle_error
			#	I2 = self.Ki[2]* self.throttle_sum_error
			#	D2 = self.Kd[2]* self.throttle_delta_error

			#	self.pitch_sum_error += self.pitch_error
			P1 = self.Kp[1] * self.error[1]

			#	self.pitch_prev_error = self.pitch_error - self.pitch_prev_error
			if self.error[1] < 3 and self.error[1] > -3:
			
				I1 = self.Ki[1] * self.sum_error[1]
			else:
				I1 = 0

			D1 = self.Kd[1] * self.delta_error[1]

			self.cmd.rcPitch = int(1500 + P1 + I1 + D1)

			if self.cmd.rcPitch > 1800:
				self.cmd.rcPitch = self.max_values[1]
			if self.cmd.rcPitch < 1200:
				self.cmd.rcPitch = self.min_values[1]

			#self.delta_error[1]= self.error[1]-self.delta_error[1]
			self.prev_error[1] = self.error[1]
			self.sum_error[1] += self.error[1]

			P = self.Kp[0] * self.error[0]
			if self.error[0] < 1.5 and self.error[0] > -1.5:
				I = self.Ki[0] * self.sum_error[0]
			else:
				I = 0

			D = self.Kd[0] * self.delta_error[0]

			self.cmd.rcRoll = int(1500 + P - I + D)

			if self.cmd.rcRoll > 1800:
				self.cmd.rcRoll = self.max_values[0]
			if self.cmd.rcRoll < 1200:
				self.cmd.rcRoll = self.min_values[0]

			#self.delta_error[0]= self.error[0]-self.delta_error[0]
			self.prev_error[0] = self.error[0]
			self.sum_error[0] += self.error[0]

			self.command_pub.publish(self.cmd)
			self.throttle_error_pub.publish(self.error[2])
			self.pitch_error_pub.publish(self.error[1])
			self.roll_error_pub.publish(self.error[0])

			t=rospy.Rate(20)
			t.sleep()


			# t1_stop = process_time()
			# print("Elapsed time during the whole program in seconds:",t1_stop-t1_start)		


				

					
				
				


		
		


		





		#------------------------------------------------------------------------------------------------------------------------


			
			# self.command_pub.publish(self.cmd)
			# self.throttle_error_pub.publish(self.error[2])
			# self.pitch_error_pub.publish(self.error[1])
			# self.roll_error_pub.publish(self.error[0])

	
	
	pid_tuned = func_wrapper(pid)





			

				
			
			


	
	


	





	#------------------------------------------------------------------------------------------------------------------------


		
		#self.command_pub.publish(self.cmd)
		#self.throttle_error_pub.publish(self.error[2])
		#self.pitch_error_pub.publish(self.error[1])
		#self.roll_error_pub.publish(self.error[0])

	#pid_tuned = func_wrapper(pid)


	
















	#----------------------------------------------------------------------------------------------------------------------


	

if  __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(20) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz #12
	while not rospy.is_shutdown():
		e_drone.pid_tuned()
		r.sleep()
		
	
	
	
		
	