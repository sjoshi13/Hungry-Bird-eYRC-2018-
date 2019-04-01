#!/usr/bin/env python


'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [-4,-2,30,.00]	
                self.start_pos=[]
		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint =[6,2,25,.00] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

                self.waypoints={}
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
	        PIDvals=[[[40,25,40, 0],[0,0,0,0],[940,875,75,0]] , [[28,28,40, 0],[0,0,0,0],[650,650,80,0]] , [[48,20,60, 0],[0,0,0,0],[1000,990,110,0]]]#600,380,800
                select=2# 0:600 1:380 2:800
                self.Kp=PIDvals[select][0]
                self.Kd=PIDvals[select][2]
                self.Ki=PIDvals[select][1]
                
              		#-----------------------Add other required variables for pid here ----------------------------------------------
                self.error=[0,0,0,0]
                self.error_sum=[0,0,0,0]
                self.error_rate=[0,0,0,0]
  
                self.previous_error=[8.39,-4.98,-27.92,0.0]
                self.max_values=[1545,1545,1800,1545]
                self.min_values=[1445,1445,1445,1480]
                self.out=[0,0,0,0]   
                


                self.goal=0#variable to change goal




		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.040 # in seconds

                self.request=False#checks if path computation is complete





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#-s-----------------------Add other ROS Publishers here-----------------------------------------------------
                self.alt_err_pub = rospy.Publisher('/alt_error', Float64 ,queue_size=1)
                self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64 ,queue_size=1)
                self.roll_err_pub = rospy.Publisher('/roll_error', Float64 ,queue_size=1)
                self.yaw_err_pub = rospy.Publisher('/yaw_error', Float64 ,queue_size=1)
                self.zero=rospy.Publisher('/zero_line',Float64,queue_size=1)
                self.getPath=rospy.Publisher('/get',UInt16,queue_size=1)


               

		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
                rospy.Subscriber('/pid_tuning_yaw', PidTune , self.yaw_set_pid)
                rospy.Subscriber('/input_key', Int16, self.key_callback)
                rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
                rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
                rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)
                rospy.Subscriber('/done_computing',UInt16,self.done_computing)
                rospy.Subscriber('/vrep/waypoints',PoseArray,self.planned)

                self.pre_error_array = [[0, 0, 0, 0]]*3
                self.drone_arm=0
                self.prev_time=time.time()
                self.previous_time=time.time()
	        self.no_of_path_points=81
                self.skip_number = 20
                self.run_to = 5 
                print('trying')	#Skipping waypoints that are too close to each other#------------------------------------------------------------------------------------------------------------
               
                # ARMING THE DRONE
                self.get_path_published=0
                self.goal_no=0
                self.total_goals=5#2n+1
             
        def start(self):
                  
                  self.pid()
                  if abs(self.error[0])<=1.5 and abs(self.error[1])<=1.5 and abs(self.error[2])<=1.5 and abs(self.error[3])<=2:#checking if the error is less than 0.5        
                     #print('requesting next')
             #        rospy.sleep(.06)
                     if self.get_path_published==0: 
                       self.goal_no+=1   #increment goal number
                       
                       if self.goal_no>self.total_goals:#Land and disarm if all three goals are reached
                           #self.landing()
                           self.disarm()
                          # break
                       self.getPath.publish(self.goal_no)
                       self.get_path_published=1#stop requesting new path till present goal is reached
                     if self.request==True:
                        print('calling ompl')
                        self.ompl_move() 
                        self.get_path_published=0      
                             
                                            
                      
        #this function runs PID for waypoint navigation on OMPL path    
        def key_callback(self,msg):
             if msg.data==60:
                   self.drone_arm=1
                      
        def ompl_move(self):
            self.request=False
            self.get_path_published=0
            print('in ompl move')
            if self.goal_no==2 or self.goal_no==4 or self.goal_no==6:
                   self.run_to = 2
            for i in range(self.run_to):       
                 while(1):
                     print(i,self.waypoints[i])
                     self.setpoint=self.waypoints[i]
                     self.pid()
                     rospy.sleep(self.sample_time)
                     if (abs(self.error[0])<=1.5 and abs(self.error[1])<=1.5 and abs(self.error[2])<=1.5 and abs(self.error[3])<=2):
                         #rospy.sleep(.06)
                         print("completed ",i)
                         break
            self.run_to = 5
            



        # This function stores the stored path in dictionary waypoints
        def planned(self,msg):
     
          index=0
          if self.goal_no==2 or self.goal_no==4 or self.goal_no==6:
                   self.skip_number = 80
          
          for i in range(0,self.no_of_path_points,self.skip_number):     #Skipping waypoints that are too close to have a reasonable distance between two drone positions/
              self.waypoints[i/self.skip_number]=[round(msg.poses[i].position.x,3),round(msg.poses[i].position.y,3),round(msg.poses[i].position.z,3),.01]
              
          self.request=True
          self.skip_number=20
          print('palnned',self.waypoints)


        def done_computing(self,msg):
                  x=msg.data
                  print("here in done_caomp",x)
                  if x==1:
                     self.request=True
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


        def landing(self):
                           self.cmd.rcRoll = 1500
		           self.cmd.rcPitch = 1500
		           self.cmd.rcYaw = 1500
	               	   self.cmd.rcThrottle = 1500
                           self.command_pub.publish(self.cmd)
                           rospy.sleep(.2)
                           self.cmd.rcThrottle = 1400
                           self.command_pub.publish(self.cmd)
                           rospy.sleep(4)
	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
                
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
               
                self.drone_position[1]=msg.poses[0].position.y
                self.drone_position[2]=msg.poses[0].position.z
               
        def yaw_callback(self,msg):
                self.drone_position[3]=msg.data
        

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
                      
              curr_time=time.time()
              if(curr_time-self.prev_time>self.sample_time):
                if(curr_time-self.previous_time>.5):#whycon is hidden
                        count=0
                        for i in range(4):
                             if self.error[i]==self.previous_error[i]:
                                   count+=1
                        if count==4:
                             for i in range(4):
                                self.error[i]=self.drone_position[i]-self.setpoint[i]
                                if self.error[i]>2:
                                 self.error[i]=2
                                self.error_rate[i]=2
                                self.out[i]=(self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i])                   
                        self.cmd.rcPitch = 1500-(self.out[0])
		        self.cmd.rcRoll = 1500-(self.out[1])
		        self.cmd.rcThrottle = 1500+self.out[2]
                        
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
                        return
                self.prev_time=curr_time
                new_list = [0]*4
                error_changed_check=0
                for i in range(4):
                 new_list[i] = self.error[i]   
                 self.error[i]=self.drone_position[i]-self.setpoint[i]
                 
                 if self.error[i]>4:
                     self.error[i]=4
		 #print("err Position",i,' : ',self.error[i])
                 for temp in range(3):
                    self.error_sum[i] += (self.pre_error_array[temp][i])
                 self.error_rate[i]=(self.error[i]-self.previous_error[i])
                 
                 self.out[i]=(self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i])
                
            
                 self.error_sum[i]=0
                 self.previous_error[i]=self.error[i]
                #if error_changed_check==4:#!
                #       return#!
                self.pre_error_array.append(new_list)
                #print(self.pre_error_array, "Updation")
                self.pre_error_array = self.pre_error_array[1:]
                #print(self.pre_error_array, "Updation")
               	#if x[0]!=True:
                self.cmd.rcPitch = 1500-(self.out[0])
		i#f x[1]!=True:
                self.cmd.rcRoll = 1500-(self.out[1])
		#self.cmd.rcYaw = 1500-self.out[3]
	
                self.cmd.rcThrottle = 1500+self.out[2]
                
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
     



if __name__ == '__main__':

	 e_drone = Edrone()
         x=0
	 while not rospy.is_shutdown():
                 if e_drone.drone_arm==1:
                  if x==0:
                    print('arming')
                    e_drone.setpoint=[e_drone.drone_position[0],e_drone.drone_position[1],25,0]#hover at the start position
                    e_drone.disarm()
                    e_drone.arm()
                    e_drone.drone_arm=0
                    x=1
		 e_drone.start()

