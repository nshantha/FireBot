#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from math import *
import tf

from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, Twist
from random import sample
from math import pow, sqrt
global initial_pose
global locations
global prev
global fire_points
def get_euclidean_distance(start_position,next_goal):
    #print(start_position,"\n\nHERE:",next_goal)
    y_diff=(next_goal.y-start_position.y)
    x_diff=(next_goal.x-start_position.x)
    distance=sqrt((x_diff)**2+(y_diff)**2)
    return distance

def update_initial_pose(initial_pose_):
    global initial_pose
    global locations
    global prev
    global fire_points

    initial_pose = initial_pose_
    
    min_=99999999
    min_point=Pose()
    

    for ele in locations:
        
        dist=get_euclidean_distance(initial_pose.pose.pose.position,ele.position)
        if min_ > dist and dist<2.0:
            min_=dist
            min_point=ele
    #print("Distance: ",min_point.position.x,min_point.position.y)
    if min_point != prev:
        fire_points.append(min_point)
        prev=min_point
        #print(min_point)


def listener():
   global locations
   global prev
   global fire_points
   rospy.init_node('init_', anonymous=False)
   rospy.Subscriber('odom',Odometry, update_initial_pose)
   firelocations = rospy.Publisher('fire_locations', Pose, queue_size=1)
   prev=Pose()
   fire_points=[]
   locations=[Pose(Point(1.75, -0.75, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0)),
            Pose(Point(4.0, 1.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0)),
            Pose(Point(5.720, 1.229, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0))]
   
   while not rospy.is_shutdown():
       
     if len(fire_points)==len(locations):
       for point in fire_points:
            print(point)

            firelocations.publish(point)
           
        


    # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
    listener()