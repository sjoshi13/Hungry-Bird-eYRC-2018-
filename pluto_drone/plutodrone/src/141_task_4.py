#!/usr/bin/env python
'''
	This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
	This node publishes and subsribes the following topics:
		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command		/whycon/poses
		/alt_error			/pid_tuning_altitude
		/pitch_error		/pid_tuning_pitch
		/roll_error			/pid_tuning_roll
		/yaw_error          /input_key
        /zero_line			/pid_tuning_yaw
		/get				/drone_yaw
 		/done_computing     /vrep/waypoints
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
			# initializing ros node with name drone_control
			rospy.init_node('drone_control')
			# This corresponds to current position of drone. This value must be updated each time in your whycon callback
			# [x,y,z,yaw_value]
			self.drone_position = [0,0,0,.00] # [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]	
            		self.start_pos=[]
			self.setpoint =[0,0,0,.00] # whycon marker at the position of the dummy given in the scene.
			#The whycon marker associated with position_to_hold dummy renderable and make changes accordingly
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
	        	
            		#-----------------------other required variables for pid ----------------------------------------------
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
			#------------------------Add other ROS Publishers here-----------------------------------------------------
            		self.alt_err_pub = rospy.Publisher('/alt_error', Float64 ,queue_size=1)
            		self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64 ,queue_size=1)
            		self.roll_err_pub = rospy.Publisher('/roll_error', Float64 ,queue_size=1)
            		self.yaw_err_pub = rospy.Publisher('/yaw_error', Float64 ,queue_size=1)
            		self.zero=rospy.Publisher('/zero_line',Float64,queue_size=1)
            		self.getPath=rospy.Publisher('/get',UInt16,queue_size=1) # publishes goal number for which path is to be planned
             
		    	#-----------------------------------------------------------------------------------------------------------
		    	# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
			rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
			rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		    	#-------------------------other ROS Subscribers here----------------------------------------------------
            		rospy.Subscriber('/pid_tuning_yaw', PidTune , self.yaw_set_pid)
            		rospy.Subscriber('/input_key', Int16, self.key_callback)
            		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
            		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
            		rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)
            		rospy.Subscriber('/done_computing',UInt16,self.done_computing)  #Vrep publishes 1 on this topic when path computation is successful.
            		rospy.Subscriber('/vrep/waypoints',PoseArray,self.planned)
            		self.break_while=0  #used to break while loop if the drone crashes. The drone is then placed at start position and continues motion
            		self.pre_error_array = [[0, 0, 0, 0]]*3 #Stores past errors in roll, pitch and throttle
            		self.drone_arm=0                        #Is set 1 when drone is to be armed
            		self.prev_time=time.time()
			self.no_of_path_points=57   #This is the number of path points published by vrep 
			#we are taking a path of 8 points and picking every 7th point starting from 0
			# 7 * 8 = 56; 56 + 1 =57
            		self.get_path_published=0 	#This variable is used to command Vrep to compute and publish next path
            		self.goal_no=0              #Used for navigating to different hoops
            		self.start_pid=-1           
            		self.skip_number = 7        # Skipping some of the Vrep points that are too close
            		self.run_to = 9             # total number of points in any path
            		self.total_goals=15         #Total number of goals set according to the number of hoops and insects
            		'''
			Function Name: start
			Function input: None
			Logic: This is the block which helps us to navigate through the arena. It calls the ompl_move funtion and also publisheson topic /get.
					
		    	'''
        def start(self):
                self.pid()                  
                if abs(self.error[0])<=.8 and abs(self.error[1])<=.8 and abs(self.error[2])<=.9 and abs(self.error[3])<=2:
				#checking if the error is less than 0.8 for x and y coordinates and less than 0.9 for z coordinate
                	if self.get_path_published==0:     
					
					self.goal_no+=1   #increment goal number
					self.getPath.publish(self.goal_no)
					
					self.get_path_published=1#stop requesting new path till present goal is reached
			if self.goal_no>self.total_goals:#disarm if all goals are reached
						self.disarm()
                        if self.result_path_vrep==True: #If Vrep has successfully computed path
                    
                    		self.ompl_move()  
                    		self.get_path_published=0  #After returning from ompl_move function new path is requested 
	'''
	Function Name: key_callback
	Function Input: msg (input key)
	Logic: This function is called everytime key is pressed on the keyboard or a value is published on /input_key topic. 
	'''		
        def key_callback(self,msg):
             if msg.data==60:       #if 's' is pressed, arm the drone for the first time
                self.start_pid=99
                    
                    self.drone_arm=1
            if msg.data==90:        #if 't' is pressed, arm the drone in between the run in case of a crash
                
                self.break_while=1  #to stop current waypoint loop
                self.getPath.publish(self.goal_no) #re-request a path to set goal from start position
                self.disarm()
                self.arm()
                      
	'''
	Function Name: ompl_move
	Function input: None
	Logic: Once the planned path points have been stored in "self.waypoints" dictionary 
				ompl_move starts the motion of drone along the waypoints 
	'''
        def ompl_move(self):
            self.result_path_vrep=False
 
            self.get_path_published=0
            if self.goal_no==2:
               self.waypoints
            if self.goal_no==
            for i in range(self.run_to):       
                while(1):
                    
                    self.setpoint=self.waypoints[i]
                    self.pid()
                    rospy.sleep(self.sample_time)
                    if (abs(self.error[0])<=.85 and abs(self.error[1])<=.85 and abs(self.error[2])<=.9 and abs(self.error[3])<=2):          
                        break
                    if self.break_while==1:#in case of a crash
                        while(self.result_path_vrep!=True):
                            i=0 #while path is not published stay in the while loop
			continue #continue the for loop with new path points
	'''
	Function Name: planned
	Function Input: Waypoints array
	Logic: This function stores the stored paths in dictionary of waypoints
	'''
        def planned(self,msg):     
			index=0
			#Skipping waypoints that are too close to have a reasonable distance between two drone positions
			for i in range(0,self.no_of_path_points,self.skip_number):     
				x=round(msg.poses[i].position.x,3)
				y=round(msg.poses[i].position.y,3)
			
				if abs(x)>=4.75 or abs(y+2.33)>=4.75: #accounts for difference in z-coordinates on the corners of the frame
					self.waypoints[i/self.skip_number]=[round(msg.poses[i].position.x,3),round(msg.poses[i].position.y,3),round(msg.poses[i].position.z,3)-1,.01]
				else:
					self.waypoints[i/self.skip_number]=[round(msg.poses[i].position.x,3),round(msg.poses[i].position.y,3),round(msg.poses[i].position.z,3),.01]
			self.result_path_vrep=True
			

	'''
	Function Name: done_computing
	Function Input: integer
	Logic: Once the path has been successfuly computed by vrep, it publishes 1   
	'''
        def done_computing(self,msg):
		x=msg.data
                if x==1:
                    self.result_path_vrep=True
	'''
	Function Name: disarm
	Function Input: None
	Logic: used to disarm the drone
	'''
 	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	'''
	Fucntion Name: arm
	Fucntion Input: None
	Logic: Used to set the values of the Roll, Pitch, Yaw, Throttle such that the drone arms itself
					and is ready for flight
			Arming condition of the drone : Best practise is to disarm and then arm the drone.
	''' 
	def arm(self):
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
		
	'''
	Function Name: Landing
	Function Input: self
	Logic: This function sets the value of Roll, Throttle, Yaw and Pitch suitable for the landing of drone
	'''
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
		
	'''
	Function Name: whycon_callback
	Function Input: msg (whycon poses)
	Logic: The function gets executed each time when /whycon node publishes /whycon/poses
	'''  
	def whycon_callback(self,msg):
	    self.drone_position[0] = msg.poses[0].position.x
            self.drone_position[1]=msg.poses[0].position.y
            self.drone_position[2]=msg.poses[0].position.z
        
	'''
	Function name: yaw_callback
	Function Input: whycon_poses
	Logic: The default yaw tuning 
	'''
        def yaw_callback(self,msg):
            self.drone_position[3]=msg.data
        
	'''
	Function Name: altitude_set_pid
	Function Input: pid value for throttle set in the gui
	Logic: Sets the Kp, Ki, Kd Values for Altitude
	'''
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.03
		
	'''
	Function Name: pitch_set_pid
	Function Input: pid value for pitch set in the gui
	Logic: Sets the Kp, Ki, Kd Values for Pitch
	'''
	def pitch_set_pid(self,pitch):
            self.Kp[0]=pitch.Kp*.006
            self.Ki[0]=pitch.Ki*.008
            self.Kd[0]=pitch.Kd*.03
			
	'''
	Function Name: roll_set_pid
	Function Input: pid value for roll set in the gui
	Logic: Sets the Kp, Ki, Kd Values for Roll
	'''
	def roll_set_pid(self,roll):    
            self.Kp[1]=roll.Kp*.006
            self.Ki[1]=roll.Ki*.00008
            self.Kd[1]=roll.Kd*.03
			
	'''
	Function Name: yaw_set_pid
	Function Input: pid value for yaw set in the gui
	Logic: Sets the Kp, Ki, Kd Values for Yaw
	'''
        def yaw_set_pid(self,yaw):
              self.Kp[3]=yaw.Kp*.006
              self.Ki[3]=-yaw.Ki*.00008
              self.Kd[3]=yaw.Kd*.03
#----------------------------------------------------------------------------------------------------------------------

	'''
	Function Name: pid
	Function Input: None
	Logic: Using error in position, the error is multiplied by the respective factor (Kp, Ki, Kd) and 
					accordingly the values of Roll, Yaw, Pitch, Throttle are set
					Function also takes care of drone commands to be published in case the whycon marker gets hidden.
	'''
	def pid(self):             
            curr_time=time.time()          
            if(curr_time-self.prev_time>self.sample_time):
                self.prev_time=curr_time
                new_list = [0]*4          #To store past errors of throttle, pitch, yaw and roll
                error_changed_check=0     
                for i in range(4):
			new_list[i] = self.error[i]   
			self.error[i]=self.drone_position[i]-self.setpoint[i]
			if self.error[i]>4:    #Limitng the error
				self.error[i]=4
			for temp in range(3): 
				self.error_sum[i]+= (self.pre_error_array[temp][i]) #adding past 3 errors
				self.error_rate[i]=(self.error[i]-self.previous_error[i])
				if self.error_rate==0 and i!=2 :         #if errors are exactly same then whycon is hidden
						self.out[i]=((self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i]))/130 #The calculated value is reduced by a factor so as to 
						                                                                                                              #prevent the drone from flying off the arena is case the whycon is hidden
				else:   
						self.out[i]=(self.Kp[i]*self.error[i])+(self.Kd[i]*self.error_rate[i])+((self.error_sum[i])*self.Ki[i])
			self.error_sum[i]=0    #initialising for next iteration
			self.previous_error[i]=self.error[i]
                self.pre_error_array.append(new_list)
                self.pre_error_array = self.pre_error_array[1:] #poping  old errors
                #Computing values to be published on drone commands
		
		self.cmd.rcPitch = 1500-(self.out[0])
                self.cmd.rcRoll = 1500-(self.out[1])
                self.cmd.rcThrottle = 1500+self.out[2]
                #published for plotjuggler to tune pid
		
		self.alt_err_pub.publish(self.error[2])
                self.pitch_err_pub.publish(self.error[0])
                self.roll_err_pub.publish(self.error[1])
                #Limiting drone command values within a range of min and max values
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

		'''
			The following is the heart of the code and it controls the series of activities that arm the drone and 
			helps us navigate through the arena
		'''
if __name__ == '__main__':
		e_drone = Edrone()
			x=0
		while not rospy.is_shutdown():
                if x==0: #executed only once at the begining of the code to arm the drone 
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
                if e_drone.start_pid==99: #if drone has been armed then start PID
			e_drone.start()
