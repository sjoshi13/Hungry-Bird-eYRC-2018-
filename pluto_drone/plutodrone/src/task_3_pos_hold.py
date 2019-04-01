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



class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	
                
		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint =[.5,.7,23,0]# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


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
	        '''
                self.Kp = [20,20,60, 0]#600mAh
		self.Ki = [0,0,0,0]
		self.Kd = [780,780,80,0]
                
                self.Kp = [28,28,40, 0]#380mAh
		self.Ki = [0,0,0,0]
		self.Kd = [650,650,80,0]
                '''
                self.Kp = [24,14,60, 0]#800mAh
		self.Ki = [0,0,0,0]
		self.Kd = [880,880,110,0]
                
		#-----------------------Other required variables for pid----------------------------------------------
                self.error=[0,0,0,0]
                self.error_sum=[0,0,0,0]
                self.error_rate=[0,0,0,0]
  
                self.previous_error=[0,0,31, 0]
                self.max_values=[1575,1575,1800,1530]
                self.min_values=[1425,1425,1420,1480]
                self.out=[0,0,0,0] 
                self.pre_error_array = [[0, 0, 0, 0]]*3# array of past errors for i-term of PID
		self.sample_time = 0.060 # in seconds






               
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
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



                self.i=0

		#------------------------------------------------------------------------------------------------------------
                self.disarm()
		self.arm() # ARMING THE DRONE
           
                self.prev_time=time.time()
               

        def start(self):
              


                 
                
                  
               
                     while(1):
                      
                      self.pid()
                      if (abs(self.error[0])<=2 and abs(self.error[1])<=2 and abs(self.error[2])<=2 and abs(self.error[3])<=2):
                         #rospy.sleep(.06)
                        
                        
                         break                
                        
                       
                 
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
                self.setpoint[3]=msg.data

		
		

	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp
		self.Ki[2] = alt.Ki
		self.Kd[2] = alt.Kd 

	
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
      
#- x is negtive roll
#+y negative pitch
              print(self.setpoint,self.error)
              curr_time=time.time()
              if(curr_time-self.prev_time>self.sample_time):
                self.prev_time=curr_time  
                new_list = [0]*4
                for i in range(4):
                
                 
                 new_list[i] = self.error[i]   
                 self.error[i]=-self.drone_position[i]+self.setpoint[i]
                 if self.error[i]>7:
                     self.error[i]=(self.error[i]-7)/7 + 7
		
                 for temp in range(3):
                    self.error_sum[i] += (self.pre_error_array[temp][i])
                 self.error_rate[i]=(self.error[i]-self.previous_error[i])
                 
                 self.out[i]=(self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i])
                
                 self.error_sum[i]=0
                 self.previous_error[i]=self.error[i]
                self.pre_error_array.append(new_list)
                
                self.pre_error_array = self.pre_error_array[1:]
               
                self.cmd.rcPitch = 1500+(self.out[0])
		
                self.cmd.rcRoll = 1500+(self.out[1])
		
	
                self.cmd.rcThrottle = 1500-self.out[2]
                self.alt_err_pub.publish(self.error[2])
                self.pitch_err_pub.publish(self.error[0])
                self.roll_err_pub.publish(self.error[1])
             
                if self.cmd.rcThrottle>self.max_values[2]:
                 self.cmd.rcThrottle=self.max_values[2]
                
                if self.cmd.rcRoll>self.max_values[1]:
                 self.cmd.rcRoll=self.max_values[1]
                
                if self.cmd.rcPitch>self.max_values[0]:
                 self.cmd.rcPitch=self.max_values[0]
                
                if self.cmd.rcThrottle<self.min_values[2]:
                 self.cmd.rcThrottle=self.min_values[2]
                
                if self.cmd.rcRoll<self.min_values[1]:
                 self.cmd.rcRoll=self.min_values[1]
                
                if self.cmd.rcPitch<self.min_values[0]:
                 self.cmd.rcPitch=self.min_values[0]
	        self.command_pub.publish(self.cmd)
                print(self.cmd)
     


if __name__ == '__main__':

	e_drone = Edrone()
      

	while not rospy.is_shutdown():
             
                
		e_drone.start()
