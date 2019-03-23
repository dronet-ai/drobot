#!/usr/bin/env python
# coding: utf-8
# license removed for brevity
# In[1]:

## This structure is based on the ros tf2 listener provided by tf2 class 
### link : http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29


#############################################       Dependencies       ###################################################
from math import *
from time import time,sleep
import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from mav_msgs.msg import RateThrust
from tf2_msgs.msg import TFMessage
import sensor_msgs.msg
##############################################      Hyperparameters    ###################################################

##### Nodes handler
class Coordinate():
    def __init__(self,x=0,y=0,z=1):
        self.x=x
        self.y=y
        self.z=z 

##### Constants
g=9.80655 # gravity
m=1 # vehicle mass = 1kg

##### Quadcopter pitch_roll_yaw-rates limitations

max_pitch_rate= pi/4 # rad/s
min_pitch_rate=-max_pitch_rate
max_theta_rate= max_pitch_rate
min_theta_rate=-max_theta_rate


max_roll_rate= pi/4 # rad/s
min_roll_rate=-max_roll_rate
max_phi_rate= max_roll_rate
min_phi_rate=-max_phi_rate


max_yaw_rate= pi/2 # rad/s
min_yaw_rate=-max_yaw_rate
max_psi_rate= max_yaw_rate
min_psi_rate=-max_psi_rate

max_thrust= 37 # 4*thrust_coeff*max_popr_speed**2
min_thrust= 0

##### Quadcopter pitch_yaw limitations (outter_loop) (roll controlled to be zero)

max_pitch= pi/4 # rad
min_pitch=-max_pitch

max_roll= pi/4 # rad
min_roll=-max_roll

max_yaw= pi # rad
min_yaw=-max_yaw



##### PIDS
#PID theta_rate
kp_theta_rate,kd_theta_rate,ki_theta_rate=(100,10,1)

#PID phi_rate
kp_phi_rate,kd_phi_rate,ki_phi_rate=(100,10,1)

#PID psi_rate
kp_psi_rate,kd_psi_rate,ki_psi_rate=(100,5,0)

#PID thrust
kp_thrust,kd_thrust,ki_thrust=(5,5*sqrt(2),0.01)

# PID pitch
#kp_pitch, kd_pitch, ki_pitch = (0.1, 0.2, 0)
kp_pitch, kd_pitch, ki_pitch = (0.12, 0.16, 0)

# PID roll
#kp_roll, kd_roll, ki_roll = (0.1, 0.2, 0)
kp_roll, kd_roll, ki_roll = (0.12, 0.16, 0)

#PID yaw
kp_yaw,kd_yaw,ki_yaw=(1,2*sqrt(1),0.01)

#### Planar constants
distance=0 # m : distance forward and behind the gate
threshold=0.2 # (drone - waypoint) < threshold ==> switch to next waypoint



############################################      Utils functions     #####################################################

def pqr_to_phidot_thetadot_psidot(p,q,r,theta,phi):
    
    phi_dot= p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
    theta_dot= q*cos(phi) - r*sin(phi)
    psi_dot= q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta) 
    
    return(phi_dot,theta_dot,psi_dot)

def toEulerAngle(i,j,k,one):
    
    q_w,q_x,q_y,q_z=one,i,j,k #modified to match quaternion ordrer given by flightGoggle
    
    #roll (x-axis rotation)
    sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
    cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
    roll = atan2(sinr_cosp, cosr_cosp);

    #pitch (y-axis rotation)
    sinp = +2.0 * (q_w * q_y - q_z * q_x);
    if (fabs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp); # use 90 degrees if out of range
    else:
        pitch = asin(sinp);

    #yaw (z-axis rotation)
    siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
    yaw = atan2(siny_cosp, cosy_cosp);
    
    return (pitch,roll,yaw)

def phidot_thetadot_psidot_to_pqr(phi_dot,theta_dot,psi_dot,theta,phi):
    p=phi_dot - sin(theta)*psi_dot
    q=cos(phi)*theta_dot + sin(phi)*cos(theta)*psi_dot
    r=-sin(phi)*theta_dot + cos(phi)*cos(theta)*psi_dot
    
    return(p,q,r)

def angle_transf(angle):
    # transform angle in radians to range [-pi,pi]
    
    angle=angle%(2*pi)
    if angle > pi:
        angle=angle-2*pi
    return angle
        
def regulate_with_yaw(roll_d, pitch_d, yaw):
    roll_ref = cos(yaw) * roll_d - sin(yaw) * pitch_d
    pitch_ref = sin(yaw) * roll_d + cos(yaw) * pitch_d
    return roll_ref, pitch_ref

def compute_error_limits(errors):
    errors=[abs(i) for i in errors]
    if 0 in errors:
        return errors
    m=max(errors)
    lims=[10*i/m for i in errors]
    #lims=[3,3,3]
    return lims  

def gate_to_waypoints(gate,distance):
    # from gate generate two waypoints 
    # Input: gate :[[x1,y1,z1],...] , distance to waypoint
    # 
    x3,y3,z3=gate[2]
    x4,y4,z4=gate[3]
    
    # center of the segment [(x3,y3);(x4,y4)]
    xc,yc=(x3+x4)/2,(y3+y4)/2
    
    # angle
    alpha=atan2(y3-y4,x3-x4)
    z=(gate[0][-1]+gate[1][-1]+gate[2][-1]+gate[3][-1])/4
    waypoint_1 = xc-distance*sin(alpha) , yc+distance*cos(alpha),z
    waypoint_2 = xc+distance*sin(alpha) , yc-distance*cos(alpha),z
    
    
    return (waypoint_1,waypoint_2)

def reorder_waypoints(waypoints):
    for i in range(0,len(waypoints)-2,2):
        
        d1=sum(([(waypoints[i][j]-waypoints[i+1][j])**2 for j in range(3)]))
        d2=sum(([(waypoints[i][j]-waypoints[i+2][j])**2 for j in range(3)]))
        
        if d2 < d1:
            a=waypoints[i+1]
            waypoints[i+1]=waypoints[i+2]
            waypoints[i+2]=a
    return waypoints
            
def to_tf(x,y,z):
    tf_data=geometry_msgs.msg.TransformStamped()
    tf_data.transform.translation.x=x  
    tf_data.transform.translation.y=y
    tf_data.transform.translation.z=z


    return tf_data 
    
############################################ Quadcopter Controller class #################################################

class PositionController:
    
    """
    InnerLoop Controller:  control pitch ,roll and yaw with pitch_rate,roll_rate and yaw_rate
    
    OutterLoop Controller: control distance error with pitch
                           control angular error with yaw
                           control roll to be "zero"
    """
    
    def __init__(self):
        
        
        #phi_rate PID errors
        self.error_phi_integral=0
        self.max_phi_integral=max_phi_rate #anti wind-up
        self.last_phi=0 #use last_pitch_rate instead last_error_pitch_rate to avoid derivative kick
        
        #theta_rate PID errors
        self.error_theta_integral=0
        self.max_theta_integral=max_theta_rate #anti wind-up
        self.last_theta=0 #use last_theta_rate instead last_error_theta_rate to avoid derivative kick
        
        #psi_rate PID errors
        self.error_psi_integral=0
        self.max_psi_integral=max_psi_rate #anti wind-up
        self.last_psi=0 #use last_thrust instead last_error_thrust to avoid derivative kick
        
        #thrust PID errors
        self.error_z_integral=0
        self.max_z_integral=max_thrust #anti wind-up
        self.last_z=0 #use last_thrust instead last_error_thrust to avoid derivative kick
        
        #pitch PID errors
        self.error_x_integral=0
        self.max_x_integral=max_pitch #anti wind-up
        self.last_x=0 #use last_x instead last_error_x to avoid derivative kick
        
        #roll PID errors
        self.error_y_integral=0
        self.max_y_integral=max_roll #anti wind-up
        self.last_y=0 #use last_y instead last_error_y to avoid derivative kick
        
        #yaw PID errors
        self.error_angle_integral=0
        self.max_angle_integral=max_yaw #anti wind-up
        self.last_angle=0 #use last_yaw instead last_error_yaw to avoid derivative kick
        
        
    def compute_phi_rate(self,desired_phi,actual_phi,delta_t):

        error_phi=desired_phi-actual_phi
        
        self.error_phi_integral+=error_phi*delta_t
        self.error_phi_integral=max(-self.max_phi_integral,min(self.error_phi_integral,self.max_phi_integral))

        phi_derivative= (self.last_phi-actual_phi)/delta_t #avoid derivative_kick
        self.last_phi=actual_phi

        phi_rate= kp_phi_rate*error_phi + ki_phi_rate*self.error_phi_integral + kd_phi_rate*phi_derivative

        return max(min_phi_rate, min(phi_rate,max_phi_rate))
        
    def compute_theta_rate(self,desired_theta,actual_theta,delta_t):
        
        error_theta=desired_theta-actual_theta
        
        self.error_theta_integral+=error_theta*delta_t
        self.error_theta_integral=max(-self.max_theta_integral,min(self.error_theta_integral,self.max_theta_integral))
        
        theta_derivative= (self.last_theta-actual_theta)/delta_t #avoid derivative_kick
        self.last_theta=actual_theta
        
        theta_rate= kp_theta_rate*error_theta + ki_theta_rate*self.error_theta_integral + kd_theta_rate*theta_derivative
        
        return max(min_theta_rate, min(theta_rate,max_theta_rate))
    
    def compute_psi_rate(self,desired_psi,actual_psi,delta_t):
        
        error_psi=desired_psi-actual_psi
        
        self.error_psi_integral+=error_psi*delta_t
        self.error_psi_integral=max(-self.max_psi_integral,min(self.error_psi_integral,self.max_psi_integral))
        
        psi_derivative= (self.last_psi-actual_psi)/delta_t #avoid derivative_kick
        self.last_psi=actual_psi
        
        psi_rate= kp_psi_rate*error_psi + ki_psi_rate*self.error_psi_integral + kd_psi_rate*psi_derivative
        
        return max(min_psi_rate, min(psi_rate,max_psi_rate))
    
    
    def compute_thrust(self,desired_z,actual_z,lim_z,delta_t):
        
        error_z=desired_z-actual_z
        #error_z=max(-lim_z,min(error_z,lim_z))
        
        self.error_z_integral+=error_z*delta_t
        self.error_z_integral=max(-self.max_z_integral,min(self.error_z_integral,self.max_z_integral))
        
        z_derivative= (self.last_z-actual_z)/delta_t #avoid derivative_kick
        self.last_z=actual_z
        
        thrust= kp_thrust*error_z + ki_thrust*self.error_z_integral + kd_thrust*z_derivative
        
        thrust+=g
        
        if cos(phi) != 0 and cos(theta) !=0:
            
            thrust/= (cos(phi)*cos(theta)/m)
        
        return max(min_thrust, min(thrust,max_thrust))
    
    def compute_pitch(self,desired_x,actual_x,lim_x,delta_t):
        
        error_x=desired_x-actual_x
        error_x=max(-lim_x,min(error_x,lim_x))
        
        self.error_x_integral+=error_x*delta_t
        self.error_x_integral=max(-self.max_x_integral,min(self.error_x_integral,self.max_x_integral))
        
        x_derivative= (self.last_x-actual_x)/delta_t #avoid derivative_kick
        self.last_x=actual_x
        
        pitch= kp_pitch*error_x + ki_pitch*self.error_x_integral + kd_pitch*x_derivative
        
        
        return max(min_pitch, min(pitch,max_pitch))
    
    def compute_roll(self,desired_y,actual_y,lim_y,delta_t):
        
        error_y=desired_y-actual_y
        error_y=max(-lim_y,min(error_y,lim_y))
        
        self.error_y_integral+=error_y*delta_t
        self.error_y_integral=max(-self.max_y_integral,min(self.error_y_integral,self.max_y_integral))
        
        y_derivative= (self.last_y-actual_y)/delta_t #avoid derivative_kick
        self.last_y=actual_y
        
        roll= kp_roll*error_y + ki_roll*self.error_y_integral + kd_roll*y_derivative
        
                    
        return max(min_roll, min(roll,max_roll))
    
    
    def compute_yaw(self,desired_angle,actual_angle,delta_t):
        
        error_angle=desired_angle-actual_angle
        
        self.error_angle_integral+=error_angle*delta_t
        self.error_angle_integral=max(-self.max_angle_integral,min(self.error_angle_integral,self.max_angle_integral))
        
        angle_derivative= (self.last_angle-actual_angle)/delta_t #avoid derivative_kick
        self.last_angle=actual_angle
        
        yaw= kp_yaw*error_angle + ki_yaw*self.error_angle_integral + kd_yaw*angle_derivative
        
        return max(min_yaw, min(yaw,max_yaw))

class Coordinate():
    def __init__(self,x=0,y=0,z=1):
        self.x=x
        self.y=y
        self.z=z
        
#################################################  ROSNode Function  ##########################################################
"""
def to_tf(x,y,z,q1,q2,q3,q4):
    tf_data=geometry_msgs.msg.TransformStamped()
    tf_data.transform.translation.x=x  
    tf_data.transform.translation.y=y
    tf_data.transform.translation.z=z
    tf_data.transform.rotation.x=q1
    tf_data.transform.rotation.x=q2
    tf_data.transform.rotation.x=q3
    tf_data.transform.rotation.x=q4

    


    return tf_data 
    
"""
def Publish_rateThrust(Thrust,roll_rate,pitch_rate,yaw_rate):
    rate_data=RateThrust()
      
    rate_data.header.stamp = rospy.Time.now()
    rate_data.header.frame_id = "uav/imu"
    rate_data.angular_rates.x=np.float64(roll_rate)
    rate_data.angular_rates.y=np.float64(pitch_rate)
    rate_data.angular_rates.z=np.float64(yaw_rate)
    rate_data.thrust.x=np.float64(0.0)
    rate_data.thrust.x=np.float64(0.0)
    rate_data.thrust.z=np.float64(Thrust)
    
    pub.publish(rate_data)

def uav_groundtruth_pose(tf_data,fused_data):
    
    x=tf_data.transform.translation.x
    y=tf_data.transform.translation.y
    z=tf_data.transform.translation.z
    q1=fused_data.transform.rotation.x
    q2=fused_data.transform.rotation.y
    q3=fused_data.transform.rotation.z
    q4=fused_data.transform.rotation.w

    return (x,y,z,q1,q2,q3,q4)



#################################################  ROSNode Main  ##########################################################
if __name__ == '__main__':
    ##### initiate the node and the publisher
    rospy.init_node('angular_control', anonymous=True)
    pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)
    ####### initiate tf buffer 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)####

    kick=True

    #### Transform each gate location as 2 waypoints, and save all the waypoints in a list
    
    gates=rospy.get_param("/uav/gate_names")
    gate_waypoints=[]
    
    for gate in gates :
        center_location=[]
        parameter="/uav/" + gate +"/nominal_location" 
        gate=rospy.get_param(parameter)
        gate_waypoints.append(gate_to_waypoints(gate,distance)[0])
        gate_waypoints.append(gate_to_waypoints(gate,distance)[1])
        
    initial_pose=rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
    Position=Coordinate(initial_pose[0],initial_pose[1],initial_pose[2])
    P_i,R_i,Y_i=toEulerAngle(initial_pose[3],initial_pose[4],initial_pose[5],initial_pose[6]) 
    Y_i=-Y_i
    initial_pose[2]+=1
    
    gate_waypoints.insert(0,initial_pose[0:3])  
    
    gate_waypoints=reorder_waypoints(gate_waypoints)
    
    
    ### Initialize controller
    controller = PositionController()

    ##### time parameers
    rate=rospy.Rate(60)
    delta_t=0.0166
    
    ##### first gate to pass
    gate_number=0
    
    
    while not rospy.is_shutdown():
        
        ######### lookup for tf data 
        
        ################# Slam calling process 
        try :
            trans = tfBuffer.lookup_transform("map", 'camera_link', rospy.Time())
            gt=tfBuffer.lookup_transform("world", 'uav/imu', rospy.Time())
        
        except: 
            trans=to_tf(Position.x,Position.y,Position.z)
	    gt=geometry_msgs.msg.TransformStamped()

        ################# IMU calling process
        try:
            fused_transform = tfBuffer.lookup_transform("world", 'fused_imu', rospy.Time())
            #rospy.Subscriber("/uav/sensors/downward_laser_rangefinder", sensor_msgs.msg.Range, callback)
        except tf2_ros.LookupException :
            Publish_rateThrust(35,0,0,0)
            rate.sleep()
            continue
        except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
            
        ########## get desired pose
        desired_pose = gate_waypoints[gate_number] # first gate
        
        desired_x,desired_y,desired_z = desired_pose[0],desired_pose[1],desired_pose[2]
        #print('desired', desired_x, desired_y, desired_z)
        ########### get the tf data
        x_s,y_s,z_s,q1,q2,q3,q4=uav_groundtruth_pose(trans,fused_transform)
        #z = Position.z
        #Frame transformation between SLAM and ground truth 
        x=x_s*cos(Y_i) + y_s*sin(Y_i) + initial_pose[0]  #18-y_s
        y=-x_s*sin(Y_i) + y_s*cos(Y_i) + initial_pose[1]   #-23 +x_s
        z= initial_pose[2] + z_s -1    #5.3 + z_s
        print(Y_i)
	########### transform tf to euler frame
        
        pitch,roll,yaw=toEulerAngle(q1,q2,q3,q4)
        pitch,roll,yaw=angle_transf(pitch),angle_transf(roll),angle_transf(yaw)
        phi,theta,psi=roll,pitch,yaw
        
        ########### OutterLoop Control
        
        error_x,error_y,error_z=desired_x-x,desired_y-y,desired_z-z
        lim_x,lim_y,lim_z=compute_error_limits((error_x,error_y,error_z))
        
        if (error_x**2+error_y**2+error_z**2)<threshold:
            gate_number+=1
            if gate_number==len(gate_waypoints):
                gate_number-=1
                
        thrust= controller.compute_thrust(desired_z, z,lim_z, delta_t)
        desired_pitch = controller.compute_pitch(desired_x,x,lim_x,delta_t)
    
        desired_roll = - controller.compute_roll(desired_y,y,lim_y,delta_t)
        
        desired_roll, desired_pitch = regulate_with_yaw(desired_roll, desired_pitch, -psi)
        
        
        desired_yaw= -pi/2
        
        desired_phi,desired_theta,desired_psi= desired_roll,desired_pitch,desired_yaw
        
        ########### InnerLoop Control
        
        phi_rate = controller.compute_phi_rate(desired_phi, phi, delta_t)
        theta_rate = controller.compute_theta_rate(desired_theta, theta, delta_t)
        psi_rate = controller.compute_psi_rate(desired_psi, psi, delta_t)
    
        ############ Convert 
        
        p, q, r = phidot_thetadot_psidot_to_pqr(phi_rate, theta_rate, psi_rate, theta, phi)
        roll_rate, pitch_rate, yaw_rate = p, q, r
        
        ############ Publish 
        Publish_rateThrust(thrust,roll_rate,pitch_rate,yaw_rate)
        """
        Position.x=x
	Position.y=y
	Position.z=z
        """
	print("GT   : ",gt.transform.translation.x)
        print("GT   : ",gt.transform.translation.y)
        print("GT   : ",gt.transform.translation.z)
        print("SLAM x:  " ,x)
        print("SLAM y : ",y)
        print("SLAM z: ", z)

        rate.sleep()


