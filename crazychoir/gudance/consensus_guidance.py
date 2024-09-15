from typing import Callable
import numpy as np

from rclpy.node import Node
from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe

from std_msgs.msg import Float64MultiArray 
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from time import sleep

class ConsensusGuidance(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, takeoff_time: float=5.0):
        super().__init__('consensus_guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
       
        self.agent_id = self.get_parameter('agent_id').value
        self.N = self.get_parameter('N').value
        self.init_pos = self.get_parameter('init_pos').value

        #za take off 
        self.goals_one = self.get_parameter('goals').value
        self.goals_one = [self.goals_one[i:i+4] for i in range(0, len(self.goals_one), 4)]


        #za start 
        ksi_1 = self.get_parameter('ksi').value
        #self.get_logger().info(str(self.ksi))
        self.ksi_1 = np.array(ksi_1).reshape(-1, 3)

        #ksi je formacija u koju se ide
        self.ksi = self.ksi_1

        #2nd formation
        ksi_2 = self.get_parameter('ksi_2').value
        #self.get_logger().info(str(self.ksi))
        self.ksi_2 = np.array(ksi_2).reshape(-1, 3)
       
        #a matrix
        a_flat = self.get_parameter('a_matrix').value
        self.a = np.array(a_flat).reshape(-1, 3)

        # Takeoff time
        self.takeoff_time = takeoff_time

        # Initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        #sve pozicije trenutne
        self.X = np.zeros((self.N, 3)) 
        self.positions_subscription = self.create_subscription(Float64MultiArray, '/my_positions_topic', self.positions_callback, 10)
        
        # Button subscriptions
        self.take_off_trigger_subscription = self.create_subscription(Empty, '/takeoff', self.start_take_off, 10)
        self.experiment_trigger_subscription = self.create_subscription(Empty, '/experiment_trigger', self.experiment_start, 10)

        # Publishers
        self.publishers_traj_params = self.create_publisher(Trajectory, 'traj_params', 1)

        self.get_logger().info('MyGuidance {} started'.format(self.agent_id))


        self.take_off = False
        self.start = False
        self.iteration = 0
        #ako je dx premalen znači da je postignuta formacija 
        #i nema smisla zvati funkciju (laptop mi se pregrijava feel free
        #zvati ovo beskonacno i ne imati ovu varijablu :) )
        self.dx_too_small = False 
        self.zamjena_formacije = True
        self.leading_time = False

        #provjeravaj if start has been pressed
        self.create_timer(2.0, self.update_calculations)

    def positions_callback(self, msg):
        """Callback to update positions from the 'positions_topic'."""
        
        self.X = np.array(msg.data).reshape(self.N, 3)

    def start_take_off(self, _):
        self.take_off = True
        self.start = False
        self.get_logger().info("Take Off Triggered")
        self.execute_goals(self.goals_one)

    def experiment_start(self, _):
        self.start = True
        self.take_off = False
        self.get_logger().info("Experiment Start Triggered")


    def execute_goals(self, goals):
        self.get_logger().info('Starting to take off')

        for goal in goals:
            goal_time = goal[0]
            goal_pos = goal[1:]
            self.goto(goal_time, goal_pos)
            print(f'Agent {self.agent_id} going to {goal_pos}')
            #sleep(goal_time)

    def goto(self, goal_time, goal_pos):
        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = goal_pos[0]
        y = goal_pos[1]
        z = goal_pos[2]
        duration_s = goal_time
        point.positions = [x, y, z]
        point.velocities = [0.0, 0.0, 0.0]  
        point.accelerations = [0.0, 0.0, 0.0]
        point.effort = [0.0, 0.0, 0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)

    def caluclate_separation(self, j):
        i = self.agent_id
        norm = (self.X[j][0] - self.X[i][0])**2 + (self.X[j][1] - self.X[i][1])**2 + (self.X[j][2] - self.X[i][2])**2
        if norm < 0.001:
            return -1
        
        return norm

    def move_formation(self, vel, dt = 1):
        new_X = self.X[self.agent_id] + vel * dt
        new_X_r = [round(num, 3) for num in new_X]
        return new_X_r


    def calculate_dx_pin(self, ksi, k = 0.6, dt = 1., separation = 0.01):
        i = self.agent_id
        dx_elem = [0, 0, 0]
        separation_elem = [0, 0, 0]
        for j in range (self.N):
            if i == j:
                continue
            dx_elem +=  self.a[i][j]  * ((self.X[j] - self.X[i]) - (self.ksi[j] - self.ksi[i]))
            norm = self.caluclate_separation(j)
            if norm == -1:
                #ako je norm onako 0 ne zelimo djeliti sa 0 
                #dolje pa samo continue
                #iako se to ne bi trebalo ikad dogoditi jer
                #to znaci da su se agenti doslovno
                #sudarili, ali ovo je samo
                #preventivna mjera od 
                #errora u kodu, a ne "problema" (sudaranja) u simulaciji
                continue

            separation_elem += (self.X[j] - self.X[i]) / norm
      
        g = 0.8
        za_leadera = (self.X[self.agent_id] - self.X[i]) * g #*g a samo je 3 letjelice pa je svufje 1
        new_X = self.X[i] + (k * dx_elem + separation * separation_elem + za_leadera) * dt

        
        new_X_rounded = [round(num, 3) for num in new_X]
        dx_elem_rounded = [round(num, 4) for num in dx_elem]
        return new_X_rounded, dx_elem_rounded 


    def calculate_dx(self, ksi, k = 0.6, dt = 1., separation = 0.01):
        i = self.agent_id
        dx_elem = [0, 0, 0]
        separation_elem = [0, 0, 0]
        for j in range (self.N):
            if i == j:
                continue
            dx_elem +=  self.a[i][j]  * ((self.X[j] - self.X[i]) - (self.ksi[j] - self.ksi[i]))
            norm = self.caluclate_separation(j)
            if norm == -1:
                #ako je norm onako 0 ne zelimo djeliti sa 0 
                #dolje pa samo continue
                #iako se to ne bi trebalo ikad dogoditi jer
                #to znaci da su se agenti doslovno
                #sudarili, ali ovo je samo
                #preventivna mjera od 
                #errora u kodu, a ne "problema" (sudaranja) u simulaciji
                continue

            separation_elem += (self.X[j] - self.X[i]) / norm

        new_X = self.X[i] + (k * dx_elem + separation * separation_elem ) * dt

        
        new_X_rounded = [round(num, 3) for num in new_X]
        dx_elem_rounded = [round(num, 4) for num in dx_elem]
        return new_X_rounded, dx_elem_rounded 

    def update_calculations(self):
        if self.start == True:
              
            goal_time = 1
            dx_elem = [0, 0, 0]
            #ne treba ovaj if, ali da se ne računa cijelo vrijeme radi pregirjavanja
            if self.dx_too_small == False:
                if self.agent_id == 1 and self.leading_time == True:
                    #pomicanje cijele formacije
                    vel = [0.0, 0.6, 0]
                    goal_pos = self.move_formation(vel)
                    self.goto(goal_time, goal_pos)
                    self.iteration = self.iteration + 1 
                    if self.iteration == 100:  #isto ne treba, meni za dušu
                        self.dx_too_small = False
                    
                else:
                    if self.leading_time == True:
                        new_X, dx_elem = self.calculate_dx_pin(self.ksi)
                    else:
                        new_X, dx_elem = self.calculate_dx(self.ksi)
                    goal_pos = new_X
                    self.goto(goal_time, goal_pos)


                    if dx_elem[0] + dx_elem[1] + dx_elem[2] < 0.1:
                        #self.get_logger().info("dx premalen")
                    
                        #DIO KODA ZA iZMJENU FORMACIJE
                        """
                        if self.zamjena_formacije == True:
                            self.get_logger().info("dx premalen")
                            elf.get_logger().info("dx premalen")      self.ksi = self.ksi_2
                            sleep(goal_time)
                            self.zamjena_formacije = False
                        else:
                            self.dx_too_small = True
                            self.get_logger().info("dx premalen")
                    """
                        #DIO KODA ZA POMICANJE FORMACIJE agent_1 je leader

                        if self.leading_time == False:
                            self.get_logger().info("dx premalen")
                            self.leading_time = True
                            sleep(goal_time)
                    

                    
                    
