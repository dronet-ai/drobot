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
import csv
import numpy as np
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Empty
from mav_msgs.msg import RateThrust
from tf2_msgs.msg import TFMessage
from flightgoggles.msg import IRMarkerArray,IRMarker
from scipy import ndimage, misc
from sensor_msgs.msg import Image

##############################################      Hyperparameters    ###################################################

##### Nodes handler


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

max_pitch= pi/12 # rad
min_pitch=-max_pitch

max_roll= pi/12 # rad
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
distance=2 # m : distance forward and behind the gate
threshold=0.2 # (drone - waypoint) < threshold ==> switch to next waypoint



############################################      Utils functions     #####################################################

######################### Read from ir markers topic ############################
def callback_csv(data,args):
    horizontal = args[0]
    vertical = args[1]

    horizontal.close()
    vertical.close()


def callback_image(data):
    #misc.imsave("test.png", data.data)
    print(type(data.data))
def callback(data,args):
    next_gate=args[1]
    rate = rospy.Rate(450)
    irmarker_data=args[0]
    i =0
    for dat in data.markers:
        landmark=dat.landmarkID.data
        #print(landmark,next_gate)
        if landmark == next_gate :
            irmarker_data[i].x=dat.x
            irmarker_data[i].y=dat.y
            irmarker_data[i].z=dat.z
            irmarker_data[i].landmarkID=dat.landmarkID.data
            irmarker_data[i].markerID=dat.markerID
            i+=1
        if i == 4 :
	    irmarker_data[0].found_gate=True
            break
	
    if i < 4 :
        irmarker_data[1].found_gate=True
    #print(i)
    #rate.sleep()


def gate_coordinates_from_image(imarker_data,next_gate):
    rate = rospy.Rate(1500)
    
    rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, callback,(imarker_data,next_gate))
    #rate.sleep()
    while not irmarker_data[0].found_gate :
        rate.sleep()
    #print(irmarker_data[1].found_gate)
    irmarker_data[0].found_gate=False    
    #for point in imarker_data :
        #print("X : {0} - Y : {1} - {2} - ID : {3}".format(point.x,point.y,point.landmarkID,point.markerID))
    return imarker_data 




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
    for i in range(len(waypoints)-2,2):
        
        d1=sum(([(waypoints[i][j]-waypoints[i+1][j])**2 for j in range(3)]))
        d2=sum(([(waypoints[i][j]-waypoints[i+2][j])**2 for j in range(3)]))
        
        if d2 < d1:
            a=waypoints[i]
            waypoints[i]=waypoints[i+1]
            waypoints[i+1]=a
    return waypoints 
            

############################################ Quadcopter Controller class #################################################
class IrmarkersData: 
    x = 0
    y = 0
    z = 0
    pass_gate=False
    landmarkID = ""
    markerID = ""
    found_gate=False
    def __str__(self):
		return 'gate : {0} - markerid :  {1}'.format(self.landmarkID, self.markerID )
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
        
        #Thrust VS PID errors
        self.error_vertical_integral = 0
        self.last_error_vertical = 0
        
        #Roll VS PID errors
        self.error_horizontal_integral = 0
        self.last_error_horizontal = 0
        
        #Image parameters
        self.image_height = 768 # FlightGoggle Camera parameters
        self.image_width = 1024 # FlightGoggle Camera parameters
        self.image_center = (self.image_width/2 , self.image_height/2) # (x_IC,y_IC)
        
        
        
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
    
    def compute_gate_center(self, gate_corners):
        # Use FlightGoggle rendered gates positions in the image plan 
        x=0
        y=0
        for gate in gate_corners :
            x+=gate.x
            y+=gate.y
        x=x/4
        y=y/4
        return (x,y) 
    def compute_err_pix(self,gate_center):
        # The error in pixel from the Image center to the Gate center
        # return (horizontal_error,vertical_error)
        # horizontal_error > 0 : gate at the right of the image ==> turn right
        # vertical_error > 0 : gate at the bottom of the image ==> decrease altiude
        
        return(gate_center[0]-self.image_center[0],gate_center[1]-self.image_center[1])
    
    def compute_thrust_VS(self,vertical_error,delta_t):
        
        error_vertical = - vertical_error / (  self.image_height/2) #scale to [-1,1]
        
        self.error_vertical_integral+=error_vertical*delta_t
        self.error_vertical_integral=max(-self.max_z_integral,
                                         min(self.error_vertical_integral,self.max_z_integral))
        
        error_vertical_derivative= (error_vertical-self.last_error_vertical)/delta_t 
        self.last_error_vertical=error_vertical
        
        thrust=kp_thrust*error_vertical + ki_thrust*self.error_vertical_integral + kd_thrust*error_vertical_derivative
        
        thrust+=g
        
        if cos(phi) != 0 and cos(theta) !=0:
            
            thrust/= (cos(phi)*cos(theta)/m)
            
        return max(min_thrust, min(thrust,max_thrust))
    
    def compute_roll_VS(self,horizontal_error,delta_t):
        
        error_horizontal = - horizontal_error/(self.image_width/2) #scale to [-1,1]

        
        self.error_horizontal_integral += error_horizontal*delta_t
        self.error_horizontal_integral=max(-self.max_y_integral,
                                           min(self.error_horizontal_integral,self.max_y_integral))
        
        error_horizontal_derivative= -(self.last_error_horizontal-error_horizontal)/delta_t
        self.last_error_horizontal=error_horizontal
        
        roll= kp_roll*error_horizontal + 0*ki_roll*self.error_horizontal_integral         + 0*kd_roll*error_horizontal_derivative
        
                    
        return max(min_roll, min(roll,max_roll))
    
    
        
#################################################  ROSNode Function  ##########################################################

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

def uav_groundtruth_pose(tf_data):
    
    x=tf_data.transform.translation.x
    y=tf_data.transform.translation.y
    z=tf_data.transform.translation.z
    q1=tf_data.transform.rotation.x
    q2=tf_data.transform.rotation.y
    q3=tf_data.transform.rotation.z
    q4=tf_data.transform.rotation.w
    
    return (x,y,z,q1,q2,q3,q4)


#################################################  ROSNode Main  ##########################################################

if __name__ == '__main__':
    ##### initiate the node and the publisher
    rospy.init_node('angular_control', anonymous=True)
    pub = rospy.Publisher('/uav/input/rateThrust',RateThrust ,queue_size=1)
    ####### initiate tf buffer 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)####
    #### Transform each gate location as 2 waypoints, and save all the waypoints in a list
    #rospy.Subscriber("/uav/camera/left/image_rect_color", Image, callback_image)    
    gates=rospy.get_param("/uav/gate_names")
    gate_waypoints=[]
    
    for gate in gates :
        center_location=[]
        parameter="/uav/" + gate +"/location" 
        gate=rospy.get_param(parameter)
        gate_waypoints.append(gate_to_waypoints(gate,distance)[0])
        gate_waypoints.append(gate_to_waypoints(gate,distance)[1])
    print(gates)    
    initial_pose=rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
    
    initial_pose[2]+=1
     
    gate_waypoints.insert(0,initial_pose[0:3])  
    
    gate_waypoints = reorder_waypoints(gate_waypoints)
    
    #### save only the two first waypoints
    gate_waypoints = gate_waypoints[0:2]
    gate_waypoints[1] = gate_waypoints[1][0],gate_waypoints[1][1]-2,gate_waypoints[1][2]
    gate_waypoints = [[18.0, -23.0, 6.3], (18.006790182275775, -5, 6.306497500000001),(17.006790182275775,5, 6.306497500000001)]
    #### Initialize controller
    controller = PositionController()

    ##### time parameers
    rate=rospy.Rate(1000)
    delta_t=0.002
    
    ##### first gate to pass
    gate_number=0
    VS = 0 #Visual Servoing OFF
    gate_index= 0 #Real gate index
    next_gate=gates[gate_index]




    X= open("x.csv","a+")
    Y=open("y.csv", "a+")
    Z=open("z.csv","a+")
    csvThrust=open("thrust.csv","a+")
    csvRoll=open("roll.csv","a+")
    horizontal= open("data.csv","a+")
    vertical=open("data_ver.csv", "a+")
    csvPitch = open("pitch.csv","a+")
    writer_x= csv.writer(X)
    writer_y=csv.writer(Y)
    writer_z=csv.writer(Z)
    writer_roll=csv.writer(csvRoll)
    writer_thrust=csv.writer(csvThrust)
    writer = csv.writer(horizontal)
    writer2 = csv.writer(vertical)
    writer_pitch = csv.writer(csvPitch)

    while not rospy.is_shutdown():
	#print(next_gate)        
        ######### lookup for tf data 
        try:

            trans = tfBuffer.lookup_transform("world", 'uav/imu', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
            
        ########## get desired pose
        desired_pose = gate_waypoints [gate_number] # first gate
        
        desired_x,desired_y,desired_z = desired_pose [0],desired_pose [1], desired_pose [2]
        
        
        ########### get the tf data
        x,y,z,q1,q2,q3,q4=uav_groundtruth_pose(trans)
        
        ########### transform tf to euler frame
        
        pitch,roll,yaw=toEulerAngle(q1,q2,q3,q4)
        pitch,roll,yaw=angle_transf(pitch),angle_transf(roll),angle_transf(yaw)
        phi,theta,psi=roll,pitch,yaw
        
        ########### OutterLoop Control
        
        error_x,error_y,error_z=desired_x-x,desired_y-y,desired_z-z
        lim_x,lim_y,lim_z=compute_error_limits((error_x,error_y,error_z))
        
        if (error_x**2+error_y**2+error_z**2)<threshold:
            if gate_number == 0:
                gate_number += 1
		irmarker_data = [IrmarkersData() for i in range(4)]
            elif gate_number == 1 and VS ==0:
                VS = 1 # Visual servoing ON   
		gate_number += 1
		print("VS begin")

        if y > 0 and VS == 1 :
            VS = 0
            gate_number = 2
            print("VS Done")
 
        if VS == 0 :
	            
            thrust = controller.compute_thrust(desired_z, z,lim_z, delta_t)

            desired_pitch = controller.compute_pitch(desired_x,x,lim_x,delta_t)
            desired_roll = - controller.compute_roll(desired_y,y,lim_y,delta_t)
            
            
        else :
	    #print(next_gate) 
            irmarker_data = [IrmarkersData() for i in range(4)]
            closest_gates_coordinates=gate_coordinates_from_image(irmarker_data,next_gate)# gate_coordinates_from_image() # *** Nabil ***
            gate_center=controller.compute_gate_center(closest_gates_coordinates)
            horizontal_error,vertical_error=controller.compute_err_pix(gate_center)
            	
	    writer.writerow([horizontal_error])
            writer2.writerow([vertical_error])




            desired_roll = - pi/12 #constant
            desired_pitch =  controller.compute_roll_VS (-horizontal_error,delta_t) 
            thrust =  controller.compute_thrust_VS (vertical_error*0.005,delta_t)



        desired_roll,desired_pitch = regulate_with_yaw(desired_roll, desired_pitch, - psi)
        
        desired_yaw = +pi/2
        
        desired_phi,desired_theta,desired_psi= desired_roll,desired_pitch,desired_yaw
        writer_roll.writerow([desired_roll])
        writer_thrust.writerow([thrust])
        writer_pitch.writerow([desired_pitch])
 
        ########### InnerLoop Control
        
        phi_rate = controller.compute_phi_rate(desired_phi, phi, delta_t)
        theta_rate = controller.compute_theta_rate(desired_theta, theta, delta_t)
        psi_rate = controller.compute_psi_rate(desired_psi, psi, delta_t)
    
        ############ Convert 
        
        p, q, r = phidot_thetadot_psidot_to_pqr(phi_rate, theta_rate, psi_rate, theta, phi)
        roll_rate, pitch_rate, yaw_rate = p, q, r
        
        ############ Publish 
        Publish_rateThrust(thrust,roll_rate,pitch_rate,yaw_rate)
	rospy.Subscriber("/uav/collision", Empty, callback_csv,(horizontal,vertical))
        rate.sleep()

        writer_x.writerow([x])
        writer_y.writerow([y])
        writer_z.writerow([z])
 
        
        
        
############################### Improvements
#scale horizontal and vertical error by : 1.depth or 2.gate altitude in pixel
# +- horizontal error
#desired yaw = 0
#change PD parameters for VS
#avoid derivative kick

