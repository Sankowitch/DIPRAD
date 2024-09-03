#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#randevu algoritam
#iz launch_params uzima samo broj robota
#i njihov pocetni oblik

import yaml
import roslaunch
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

robot_positions = {}
publishers = {}
adjacency_matrix = None
num_of_robots = 0





def calculate_dr(a, r, control_gain=0.1, s = 0.1):
    dr = np.zeros_like(r)
    n = len(r)
    for i in range(n):
        dr_elem = np.zeros(2)
        sep_elem = np.zeros(2)
        for j in range(n):
            if i == j:
                continue
            dr_elem += a[i][j] * (r[j] - r[i])
            sep_elem +=  (r[j] - r[i])/ np.linalg.norm((r[j] - r[i])) **2
        dr[i] = dr_elem * control_gain - s * sep_elem
    return dr

def update_r(r, dr, dt):
    return r + dr * dt



def move_robots():
    global adjacency_matrix, num_of_robots

    rate = rospy.Rate(10)  # 10 Hz
    dt = 0.1
    control_gain = 0.5

    while not rospy.is_shutdown():
        
        if len(robot_positions) < num_of_robots:
            rate.sleep()
            continue
        
        r = np.array([robot_positions[f"robot_{i}"] for i in range(num_of_robots)])
        dr = calculate_dr(adjacency_matrix, r, control_gain)
        r = update_r(r, dr, dt)

        for i in range(num_of_robots):
            robot_name = f"robot_{i}"
            twist = Twist()
            twist.linear.x = dr[i][0]
            twist.linear.y = dr[i][1]
            publishers[robot_name].publish(twist)



        rate.sleep()


def callback(msg, robot_name):
    pos = msg.pose.pose.position
    
    robot_positions[robot_name] = np.array([pos.x, pos.y])

def main():
    global adjacency_matrix, num_of_robots
    
    package = rospkg.RosPack().get_path('sphero_stage_diff')
    with open(package + '/launch/launch_params.yaml', 'r') as stream:
        config = yaml.full_load(stream)
    
    
    num_of_robots = config['num_of_robots']

    adjacency_matrix = np.ones((num_of_robots, num_of_robots)) - np.eye(num_of_robots)
    print(adjacency_matrix)

    for i in range(num_of_robots):
        robot_name = f"robot_{i}"
        odom_topic = f"/{robot_name}/odom"
        cmd_vel_topic = f"/{robot_name}/cmd_vel"
        
        rospy.Subscriber(odom_topic, Odometry, callback, callback_args=robot_name)
        publishers[robot_name] = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    rospy.sleep(2)  
    
    move_robots()

    

    

if __name__ == '__main__':
    rospy.init_node('move_my_robs')
    print("------sudar!!!-------\n")
    main()
    