def func_wrapper(func):
    
    def inner_func():
        waypoints=[[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[0,0,23]]
        #n global bana bsdk
        coord=waypoints[n]
        
        func(coord=coord)

        n+=1
    return inner_func

def timer_callback(event):
    rospy.loginfo("In timer callback")

rospy.Timer(rospy.Duration(1), timer_callback)
rospy.spin() # don't forget to spin or else your node will exit












def pid(self,coord):


		x_setpoint = coord[0]
		y_setpoint = coord[1]
		z_setpoint = coord[2]

		# #print(coord)

		# x_max = x_setpoint + 0.1
		# y_max = y_setpoint + 0.1
		# z_max = z_setpoint + 0.1

		# x_min = x_setpoint - 0.1
		# y_min = y_setpoint - 0.1
		# z_min = z_setpoint - 0.1

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

		# while (x_max < self.drone_position[0] or self.drone_position[0] < x_min) or (y_max < self.drone_position[1] or y_min > self.drone_position[1]) or (z_max < self.drone_position[2] or z_min > self.drone_position[2]):

		self.error[2] = -(coord[2] - self.drone_position[2])
		self.error[0] = (coord[0] - self.drone_position[0])
		self.error[1] = -(coord[1] - self.drone_position[1])
		self.delta_error[2] = self.error[2] - self.prev_error[2]
		self.delta_error[1] = self.error[1] - self.prev_error[1]
		self.delta_error[0] = self.error[0] - self.prev_error[0]

		print(coord)
		# print(z_max, z_min)
		print(self.error[2])
		print(self.error[1])
		print(self.error[0])

		#print( x,y,z)
		print("next")

		#print('inside')
		#print(self.error)

		P2 = self.Kp[2] * self.error[2]
		F = 0
		#print(P2)
		#if self.error[2]<3 and self.error[2]>-3:
		I2 = self.Ki[2] * self.sum_error[2]
		#print(I2)
		#	F= I2
		#else: I2 = F

		D2 = self.Kd[2] * self.delta_error[2]
		#print(D2)

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



			

				
			
			


	
	


	





	#------------------------------------------------------------------------------------------------------------------------


		
		#self.command_pub.publish(self.cmd)
		#self.throttle_error_pub.publish(self.error[2])
		#self.pitch_error_pub.publish(self.error[1])
		#self.roll_error_pub.publish(self.error[0])

pid_tuned = func_wrapper(pid)

pid_tuned()