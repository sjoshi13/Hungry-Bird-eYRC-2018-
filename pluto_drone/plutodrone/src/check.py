#!/usr/bin/env python
# Importing the required libraries
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import select

#sample time .06--->.03

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint =[0,0,20.0,0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [27.78,20.26,64.5, 0]
		self.Ki = [0,0,0,0]
		self.Kd = [65.245,577.65,617.59,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------
                self.error=[0,0,0,0]
                self.error_sum=[0,0,0,0]
                self.error_rate=[0,0,0,0]
  
                self.previous_error=[0,0,0, 0]
                self.max_values=[1575,1575,1800,1530]
                self.min_values=[1425,1425,1200,1480]
                self.out=[0,0,0,0] 
                self.pre_error_array = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]  

                self.rate=rospy.Rate(10)








		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.030 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
                self.alt_err_pub = rospy.Publisher('/alt_error', Float64 ,queue_size=1)
                self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64 ,queue_size=1)
                self.roll_err_pub = rospy.Publisher('/roll_error', Float64 ,queue_size=1)
                self.yaw_err_pub = rospy.Publisher('/yaw_error', Float64 ,queue_size=1)
                self.zero=rospy.Publisher('/zero_line',Float64,queue_size=1)





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
                rospy.Subscriber('/pid_tuning_yaw', PidTune , self.yaw_set_pid)
                rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
                rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
                rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)





		#------------------------------------------------------------------------------------------------------------
                self.disarm()
		self.arm() # ARMING THE DRONE
           
                prev_time=time.time()
                while(1):
              
                  self.pid()
             
                
                 


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
                
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
               
                self.drone_position[1]=msg.poses[0].position.y
                self.drone_position[2]=msg.poses[0].position.z

        def yaw_callback(self,msg):
                self.drone_position[3]=msg.data
        

		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp  # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 1
		self.Kd[2] = alt.Kd * 1

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

        def pitch_set_pid(self,pitch):
              self.Kp[0]=pitch.Kp
              self.Ki[0]=pitch.Ki
              self.Kd[0]=pitch.Kd
        def roll_set_pid(self,roll):    
              self.Kp[1]=roll.Kp
              self.Ki[1]=roll.Ki
              self.Kd[1]=roll.Kd
        
        def yaw_set_pid(self,yaw):
              self.Kp[3]=yaw.Kp
              self.Ki[3]=-yaw.Ki
              self.Kd[3]=yaw.Kd

       
       
       
          
         
                 
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	    ex=self.setpoint[0]-self.drone_position[0]
            ey=self.setpoint[1]-self.drone_position[1]
            ez=self.setpoint[2]-self.drone_position[2]
            curr_time=time.time()
            dt=curr_time-self.prev_time
            if(dt< self.sample_time):
                return 
            else:
              self.prev_time=curr_time
              self.out[1] = (self.Kp[1] * ey)  +  (self.Kd[1] * (ey- self.previous_error[1]))
	      self.out[0] = (self.Kp[0] * ex) + (self.Kd[0] *(error_x - self.previous_error[0]))
	      self.out[2] = (self.Kp[2] * ez)  + (self.Kd[2] * (ez - self.previous_error[2]))
              self.cmd.rcRoll= 1500  + self.out[1]
	      self.cmd.rcPitch =1500 + self.out[0]
	      self.cmd.rcThrottle = 1500 - self.out[2]


if __name__ == '__main__':

	e_drone = Edrone()
      

	while not rospy.is_shutdown():
		e_drone.pid()
