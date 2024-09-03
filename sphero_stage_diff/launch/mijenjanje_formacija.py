#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#mijenjaju oblike
#uz nesudaranje i sudaranje

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
separation = 0.0
dt = 0.1
rate_srp = 10



"""
x -> trenutni polozaj agenata
ksi -> ocekivan oblik kojeg agenti trebaju tvoriti
nesudaranje = 0 (nema varijable koja kontorlira sudaranje) | = 1 (kontrola nesudaranja ukljucena)
"""
def calculate_dx(x, ksi, nesudaranje, k = 1.0):
    global adjacency_matrix, separation
    a = adjacency_matrix
    dx =  np.zeros_like(x)
    
    n = len(x)

    if nesudaranje == False:
        separation = 0.0
        
    
    for i in range(n):
        
        dr_elem = [0, 0]
        separation_elem = [0, 0]
        
        for j in range(n):
            if i == j:
                continue
            dr_elem +=  a[i][j]  * ((x[j] - x[i]) - (ksi[j] - ksi[i]))
            #(x_j - x_i)/ |x_j - x_i|**2
            #norm = np.linalg.norm(x[j] - x[i])
            norm = (x[j][0] - x[i][0])**2 + (x[j][1] - x[i][1])**2
            separation_elem += (x[j] - x[i]) / norm

        dx[i] = k * dr_elem - separation * separation_elem 
        


    return dx

#mijenja sve koreografije iz liste u numpy
def change_to_numpy(koreografije, num_of_robots):
    koreografije_numpy = []
    for koreografija_i in koreografije:
        ksi_x = koreografija_i['x']
        ksi_y = koreografija_i['y']
        ksi = np.array([[ksi_x[i], ksi_y[i]] for i in range(num_of_robots)])
        ksi = ksi.astype(float)
        koreografije_numpy.append(ksi)

    return koreografije_numpy


"""
koreografije = [['x': [float, ..], 'y':[float, ...]], ...]
izmijenjivanje = True (nakon sto se izmjene svi oblici, pocinje se ispocetka sa prvim oblikom) | False (nakon izmjene oblike, ostaje se na zadnjem obliku)
"""
def move_robots(koreografije, izmijenjivanje, no_crashing):
    global adjacency_matrix, num_of_robots, dt, rate_srp

    index_koreografije = 0
  
    koreografije_numpy = change_to_numpy(koreografije, num_of_robots)
    ksi = koreografije_numpy[index_koreografije]

    
    vise_koreografija = False
    if len(koreografije) > 1:
        vise_koreografija =  True

    rate = rospy.Rate(rate_srp)  # 10 Hz
    control_gain = 0.4

    inter = 0

    while not rospy.is_shutdown():
        
        if len(robot_positions) < num_of_robots:
            rate.sleep()
            continue
        
        x = np.array([robot_positions[f"robot_{i}"] for i in range(num_of_robots)])
        dx = calculate_dx(x, ksi, no_crashing, control_gain)
        x = x + dx * dt
        

        for i in range(num_of_robots):
            robot_name = f"robot_{i}"
            twist = Twist()
            twist.linear.x = dx[i][0]
            twist.linear.y = dx[i][1]
            publishers[robot_name].publish(twist)

        inter = inter + 1

        #izmjenjuj oblike
        if (vise_koreografija == True) and (inter % 50 == 0): #mijenjaj koreografiju svakih 100 * dt sekundi
            index_koreografije = index_koreografije + 1
            
            if index_koreografije == len(koreografije):   #prosli smo kroz sve koreografije
                if izmijenjivanje == True: 
                    index_koreografije = 0 
                          
                else:
                    #ako smo prosli kroz sve koreografije, a
                    #izmjenjivanje je iskljuceno
                    #ostacemo na zadnjoj koreografiji 
                    vise_koreografija = False
                    
                    continue   #mozda cemo propusiti jednu iteraciju ali nema veze jer i onako stojimo u tom obliku
                    
                    
            
            ksi = koreografije_numpy[index_koreografije]

        rate.sleep()


def callback(msg, robot_name):
    pos = msg.pose.pose.position
    
    robot_positions[robot_name] = np.array([pos.x, pos.y])

def main():
    global adjacency_matrix, num_of_robots, separation, dt, rate_srp
    
    package = rospkg.RosPack().get_path('sphero_stage_diff')
    with open(package + '/launch/launch_params.yaml', 'r') as stream:
        config = yaml.full_load(stream)
    
    
    num_of_robots = config['num_of_robots']
    rate_srp = config['rate']
    dt = config['dt']

    type_of_choreography = config['choreography']
    izmijenjivanje = config['alternation']
    no_crashing = config['no_crashing']
    separation = config['separation']
    
    

    koreografije = []
    for oblik in type_of_choreography:
        koreografija_n = config['choreographies'][oblik]
        koreografije.append(koreografija_n)

    
    #asocijativna matrica je potpuno povezana; matrica 0
    adjacency_matrix = np.ones((num_of_robots, num_of_robots)) - np.eye(num_of_robots)

    #za 4 agenta ralicite matrice - 1<->4->3<->2<->1; matrica 1
    #adjacency_matrix = np.array([[0, 1, 0, 1],[1, 0, 1, 0],[0, 1, 0, 1],[1, 0, 1, 0]])

    #za 4 agenta ralicite matrice - 1->4->3->2->1; matrica 2
    #adjacency_matrix = np.array([[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1],[1, 0, 0, 0]])
    

    for i in range(num_of_robots):
        robot_name = f"robot_{i}"
        odom_topic = f"/{robot_name}/odom"
        cmd_vel_topic = f"/{robot_name}/cmd_vel"
        
        rospy.Subscriber(odom_topic, Odometry, callback, callback_args=robot_name)
        publishers[robot_name] = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    rospy.sleep(2)  
    
    move_robots(koreografije, izmijenjivanje, no_crashing)

    

    

if __name__ == '__main__':
    rospy.init_node('konsenzus_protokol_simulacija')
    print("------konsenzus protokol-------\n")
    print("------mijenjanje oblika-------\n")
    main()