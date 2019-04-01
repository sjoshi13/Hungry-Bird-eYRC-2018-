#!/usr/bin/env python
'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error			/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error			/pid_tuning_roll
		/yaw_error                      /input_key
                /zero_line			/pid_tuning_yaw
		/get				/drone_yaw
 						/done_computing
                                                /vrep/waypoints
	
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
		self.setpoint =[0,0,0,.00] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
                self.waypoints={} #This dictionary holds waypoints published by vrep
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
		


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
	        '''
                self.Kp = [35,25,45, 0]#600mAh
		self.Ki = [0,0,0,0]
		self.Kd = [1180,875,75,0]
                
                self.Kp = [35,28,40, 0]#380mAh
		self.Ki = [0,0,0,0]
		self.Kd = [750,700,80,0]
                '''
                self.Kp = [42,23,60, 0]#800mAh
		self.Ki = [0,0,0,0]
		self.Kd = [1115,950,110,0]
	        
              	#-----------------------Add other required variables for pid here ----------------------------------------------
                self.error=[0,0,0,0]
                self.error_sum=[0,0,0,0]
                self.error_rate=[0,0,0,0]
  
                self.previous_error=[0,0,0,0]
                self.max_values=[1575,1575,1800,1575]
                self.min_values=[1425,1425,1425,1480]
                self.out=[0,0,0,0]   
                

		self.sample_time = 0.040 #Sample time in seconds

                self.result_path_vrep=False#checks if path computation is complete





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
                self.break_while=0  #used to break while loop if the drone crashes. The drone is then placed at start position and continues motion
                self.pre_error_array = [[0, 0, 0, 0]]*3 #Stores past errors in roll, pitch and throttle
                self.drone_arm=0                        #Is set 1 when drone is to be armed
                self.prev_time=time.time()
	        self.no_of_path_points=57                #This is the number of path points published by vrep
                self.get_path_published=0 		#This variable is used to command Vrep to compute and publish next path
                self.goal_no=0
                self.start_pid=-1
                self.skip_number = 7
                self.run_to = 9
                self.total_goals=11#2n+1
             
        def start(self):
                 # print(self.error)
                  self.pid()
                  
                  if abs(self.error[0])<=.8 and abs(self.error[1])<=.8 and abs(self.error[2])<=.9 and abs(self.error[3])<=2:#checking if the error is less than 0.5     
                     #print("req")
             #        rospy.sleep(.06)
                     if self.get_path_published==0:     
                       print("yes getpathpub=0'")
                       self.goal_no+=1   #increment goal number
                       
                       self.getPath.publish(self.goal_no)
                       
                       print('published')
                       self.get_path_published=1#stop requesting new path till present goal is reached
                       if self.goal_no>self.total_goals:#Land and disarm if all three goals are reached
                           #self.landing()
                           self.disarm()
                          # break
                     if self.result_path_vrep==True:
                        print('calling ompl')
                        self.ompl_move()  
                        self.get_path_published=0      
                             
                                            
                      
        #this function runs PID for waypoint navigation on OMPL path    
        def key_callback(self,msg):# press s then x
             if msg.data==60:
                    self.start_pid=99
                    print("doing arm")
                    self.drone_arm=1
             if msg.data==90:
                  print('90')
                  self.break_while=1
                  self.getPath.publish(self.goal_no)
                  self.disarm()
                  self.arm()
                      
        
        def ompl_move(self):
            self.result_path_vrep=False
            self.get_path_published=0
            print('in ompl move')
            if self.goal_no==2 or self.goal_no==4 or self.goal_no==6 or self.goal_no==8 or self.goal_no==10 :
                   self.run_to = 2
                   print(self.waypoints)
            else:
                   self.run_to = 9
            for i in range(self.run_to):       
                 while(1):
                     #rint(i,self.waypoints[i])
                     self.setpoint=self.waypoints[i]
                     self.pid()
                     rospy.sleep(self.sample_time)
                     if (abs(self.error[0])<=.85 and abs(self.error[1])<=.85 and abs(self.error[2])<=.9 and abs(self.error[3])<=2):
                         #rospy.sleep(.06)
                         print("completed ",i)
                         break
                     if self.break_while==1:
                            while(self.result_path_vrep!=True):
                                     i=0
                            continue
                     
        




        # This function stores the stored path in dictionary waypoints
        def planned(self,msg):
     
          index=0
          #print('getting waypoints')
          if self.goal_no==2 or self.goal_no==4 or self.goal_no==6 or self.goal_no==8 or self.goal_no==10 :
                  self.skip_number = 56
          else:
                   self.skip_number = 7
          for i in range(0,self.no_of_path_points,self.skip_number):     #Skipping waypoints that are too close to have a reasonable distance between two drone positions/
              x=round(msg.poses[i].position.x,3)
              y=round(msg.poses[i].position.y,3)
              print(i)
              if abs(x)>=4.75 or abs(y+2.33)>=4.75:
                self.waypoints[i/self.skip_number]=[round(msg.poses[i].position.x,3),round(msg.poses[i].position.y,3),round(msg.poses[i].position.z,3)-1,.01]
              else:
                self.waypoints[i/self.skip_number]=[round(msg.poses[i].position.x,3),round(msg.poses[i].position.y,3),round(msg.poses[i].position.z,3)-1,.01]
                 
              
         # if (self.waypoints[4][1]-self.waypoints[3][1])>0:
          #   self.waypoints[5]=[self.waypoints[4][0],self.waypoints[4][1]+2,self.waypoints[4][2],0.01]
         # elif (self.waypoints[4][1]-self.waypoints[3][1])<0:
             
          #   self.waypoints[5]=[self.waypoints[4][0],self.waypoints[4][1]-2,self.waypoints[4][2],0.01]
          self.result_path_vrep=True
          
          print('palnned',self.goal_no,self.waypoints)


        def done_computing(self,msg):
                  x=msg.data
                  print("here in done_comp",x)
                  if x==1:
                     self.result_path_vrep=True
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
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.03



        def pitch_set_pid(self,pitch):
              self.Kp[0]=pitch.Kp*.006
              self.Ki[0]=pitch.Ki*.008
              self.Kd[0]=pitch.Kd*.03
        def roll_set_pid(self,roll):    
              self.Kp[1]=roll.Kp*.006
              self.Ki[1]=roll.Ki*.00008
              self.Kd[1]=roll.Kd*.03
        
        def yaw_set_pid(self,yaw):
              self.Kp[3]=yaw.Kp*.006
              self.Ki[3]=-yaw.Ki*.00008
              self.Kd[3]=yaw.Kd*.03

       
       
       
       
         
          
                    
	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
                     
              curr_time=time.time()
              
              if(curr_time-self.prev_time>self.sample_time):
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
                 if self.error_rate==0 and i!=2 :
                   self.out[i]=((self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i]))/130
                 else:   
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
                self.alt_err_pub.publish(self.error[2])
                self.pitch_err_pub.publish(self.error[0])
                self.roll_err_pub.publish(self.error[1])
             #   self.yaw_err_pub.publish(self.error[3])
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
               
		print(self.cmd)
	        self.command_pub.publish(self.cmd)
                #print(self.setpoint)



if __name__ == '__main__':

	 e_drone = Edrone()
         x=0
	 while not rospy.is_shutdown():
                if x==0:
                #   print('x e hu')
                   if e_drone.drone_arm==1:
                    print('arming')
                    e_drone.setpoint=[e_drone.drone_position[0],e_drone.drone_position[1],24,0]#hover at the start position
                    e_drone.disarm()
                    e_drone.arm()
                    e_drone.goal_no=0
                    e_drone.drone_arm=0
                    e_drone.get_path_published=0   
                    print(e_drone.setpoint)
                    x=1
                if e_drone.start_pid==99:
		  e_drone.start()
