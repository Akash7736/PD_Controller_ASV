import rospy
import tf
import math
from pyproj import Geod
import numpy as np
# tf package for conversion of quaternion to Euler angles
# Quaternion conversion will give roll, pitch and yaw
# Thrust allocation matrix which is specific for the vehicle 
# TA matrix is 1*4 matrix and can be found by Finding distance of thrusters from COG
# Smallest signed angle (ssa)
# Error matrix will be 3*1 matrix
# Error matrix contains x, y ,yaw 
# Error matrix has to be modified with ssa of yaw (smallest yaw angle) 
# Error matrix will be formed by  taking difference of desired x,y ,yaw and current x,y,yaw
# Feed Error matrix into the PD controller and get a matrix 
# Multiply controller matrix with TA matrix  ,we will get a 4*1 matrix
# PD controller is of form -Kp*e - Kd*r
# Gps will be giving x,y,z cordinates
# IMU will be giving the quaternion for roll , pitch and yaw
# In our case we need only x,y,yaw
# Find out TA matrix is formed exact calculation
# Station keeping is basically alligning with our goal position


from gpsd_client.msg import GpsFix
from sbg_driver.msg import SbgGpsPos

# def convert_to_xy(latitude, longitude):
#     in_proj = pyproj.Proj(proj='latlong', datum='WGS84')
#     out_proj = pyproj.Proj(proj='utm', zone=33, datum='WGS84')
#     x, y = pyproj.transform(in_proj, out_proj, longitude, latitude)
#     return x, y

g = Geod(ellps ='WGS84') # convert earth long and lat to 2D cordinates
or_lat = 2 # origin lat , reference 
or_lon = 6 # origin long
desired_x = 56
desired_y = 78
desired_yaw = 23
TA_matrix = np.array([[1,4,5],[4,7,9],[2,3,4],[6,8,9]])



def gps_callback(data):
    #rospy.loginfo("Received GPS data: Latitude: %f, Longitude: %f", data.latitude, data.longitude)
    my_lat = data.latitude  # lat and long to convert
    my_lon = data.longitude
    asd = g.inv(or_lon,or_lat,my_lon,my_lat)
    global x 
    x = asd[0]
    global y 
    y = asd[1]

def imu_callback(data):
    quaternion = data
    euler = tf.transformations.euler_from_quaternion(quaternion) # quaterion to euler angle conversion
    global yaw 
    yaw = euler[2] # roll, pitch , yaw
    #rospy.loginfo("Received IMU data: Yaw: %f", yaw)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/gps/fix", GpsFix, gps_callback)
    rospy.Subscriber("/sbg/gps_pos", SbgGpsPos, imu_callback)
  

error_x = desired_x - x
error_y = desired_y - y
error_yaw = desired_yaw - yaw
error_matrix = np.array([error_x,error_y,error_yaw])

class PDController:
    def __init__(self,kp,kd):
        self.kp = kp 
        self.kd = kd 
        self.lasterror = 0
        self.last_time = 0

    def update(self,error):
        present_time = time.time()
        dt = present_time - self.last_time # change in time
        derivative = (error- self.lasterror)/dt # rate of change of error
        self.lasterror = error
        self.last_time = present_time
        return (self.kp * error) + (self.kd * derivative)

PDC = PDController(0.3,0.6)
controller_matrix = []
for i in error_matrix:
    PDC.update(i)
    controller_matrix.append(i)

Thrust_alloc = np.dot(TA_matrix , controller_matrix)

if __name__ == '__main__':
    listener()
    print(Thrust_alloc)
    rospy.spin()
 



