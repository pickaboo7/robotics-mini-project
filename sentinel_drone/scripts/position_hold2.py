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

from ctypes.wintypes import PCHAR
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
		self.setpoint = [2,0,23] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

	
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
		self.Kp = [21,21,45] #0.06
		self.Ki = [0,0,0.9] #0.001
		self.Kd = [380,380,950] #0.3

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error = [0,0,0]
		self.prev_error = [0,0,0]
		self.sum_error = [0,0,0]
		self.delta_error = [0,0,0]

		#self.minthrottle = 1000
		#self.maxthrottle = 2000 

		







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
		self.cmd.rcThrottle = 1000
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
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.001
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06
		self.Ki[1] = pitch.Ki * 0.001
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki *0.001
		self.Kd[0] = roll.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------


















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
		#t1_start = process_time() 

		self.error[2] = -(self.setpoint[2] - self.drone_position[2])
		self.error[0] =  (self.setpoint[0] - self.drone_position[0])
		self.error[1] =  -(self.setpoint[1] - self.drone_position[1])
		self.delta_error[2] = self.error[2] - self.prev_error[2]
		self.delta_error[1] = self.error[1] - self.prev_error[1]
		self.delta_error[0] = self.error[0] - self.prev_error[0]

		P2 = self.Kp[2]* self.error[2]
		F=0
		I2 = self.Ki[2]* self.sum_error[2]
		
		#if self.sum_error[2]<3 and self.sum_error[2]>-3:
			#I2 = self.Ki[2]* self.sum_error[2]
			#F= I2
		#else: I2 = F

		D2 = self.Kd[2]* self.delta_error[2]

		print(self.Kd[2])
		print(D2)

		self.cmd.rcThrottle = int(1500 + P2 + I2 + D2) # (P2 + I2 + D2)

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
		P1 = self.Kp[1]* self.error[1]

	#	self.pitch_prev_error = self.pitch_error - self.pitch_prev_error
		if self.error[1] < 3 and self.error[1]>-3:
			I1 = self.Ki[1]* self.sum_error[1]
		else:
			I1 = 0	

		D1 = self.Kd[1]* self.delta_error[1]

		self.cmd.rcPitch = int(1500 + P1 + I1 + D1) 

		if self.cmd.rcPitch > 1800:
			self.cmd.rcPitch = self.max_values[1]
		if self.cmd.rcPitch < 1200:
			self.cmd.rcPitch = self.min_values[1]

		#self.delta_error[1]= self.error[1]-self.delta_error[1]
		self.prev_error[1] = self.error[1]
		self.sum_error[1] += self.error[1] 


	
	
		P = self.Kp[0]* self.error[0]
		if self.error[0] < 1.5 and self.error[0]>-1.5:
			I = self.Ki[0]* self.sum_error[0]
		else:
			I = 0

		D = self.Kd[0]* self.delta_error[0]

		self.cmd.rcRoll = int(1500 + P - I + D) 


		if self.cmd.rcRoll > 1800:
			self.cmd.rcRoll = self.max_values[0]
		if self.cmd.rcRoll < 1200:
			self.cmd.rcRoll = self.min_values[0]

		#self.delta_error[0]= self.error[0]-self.delta_error[0]
		self.prev_error[0] = self.error[0]
		self.sum_error[0] += self.error[0] 



	#	self.roll_sum_error += self.roll_error
	#	P0 = self.Kp[0]* self.roll_error
	##	I0 = self.Ki[0]* self.roll_sum_error
	#	D0 = self.Kd[0]* self.roll_dealta_error

	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

	
	


	





	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)
		self.throttle_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[1])
		self.roll_error_pub.publish(self.error[0])

		# t1_stop = process_time()
		# print("Elapsed time during the whole program in seconds:",t1_stop-t1_start) 

if  __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
