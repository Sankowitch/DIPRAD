import os
import numpy as np
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher


def get_webots_driver_cf(agent_id):
    package_dir_driver = get_package_share_directory('crazychoir_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'crazyflie_vel.urdf')).read_text()
    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace=f'agent_{agent_id}',
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':f'agent_{agent_id}',
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return crazyflie_driver

def generate_webots_world_file(robots, source_filename, target_filename):
    with open(source_filename, 'r') as source_file:
        contents = source_file.read()

    with open(target_filename, 'w') as target_file:
        target_file.write(contents)

        for robot in robots:
            template_filename = os.path.join(os.path.dirname(source_filename), f'obj_{robot["type"]}.wbt')
            with open(template_filename, 'r') as template_file:
                template = template_file.read()
                template = template.replace('$NAME', robot["name"])
                template = template.replace('$X', str(robot["position"][0]))
                template = template.replace('$Y', str(robot["position"][1]))
                template = template.replace('$Z', str(robot["position"][2]))
                target_file.write(template)


def generate_launch_description():

    N = 3

    P = np.zeros((N, 3))

    # Turltebots init positions
    P[0] = np.array([-0.4,  0.5, 0.0])
    P[1] = np.array([ 0.2,  0.5, 0.0])
    P[2] = np.array([ 0.8,  0.5, 0.0])
    
    #za uzlijetanje
    goals = {
    # id: [[t, x, y, z], ...]
        0: [[5,-0.4,  0.5, 1.0]],
        1: [[5, 0.2,  0.5, 1.0]],
        2: [[5, 0.8,  0.5, 1.0]],
       
    }

    #formacije
    ksi = [
        [-0.65, 0.1, 1.0],
        [ 0.2,  1.2, 1.0],
        [ 1.1,  0.1, 1.0],
    ]  
    
    ksi_flatten = [item for sublist in ksi for item in sublist]

    adjecancy_matrix = [
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0],
    ]
   
    a_matrix_flat = [item for sublist in adjecancy_matrix for item in sublist]
    print(a_matrix_flat)

    launch_description = []


    # Generate Webots world file
    robots = [{
            'name': f'agent_{i}',
            'type': 'crazyflie', 
            'position': P[i, :].tolist(), 
        } for i in range(N) ]
    world_package_dir = get_package_share_directory('crazychoir_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'my_world.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=target_filename)
    launch_description.append(webots)

    # Launch control Panel
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_my_formation_webots_gui',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))

    # get positions
    launch_description.append(Node(
                package='crazychoir_examples', 
                executable='crazychoir_my_formation_webots_get_positions',
                output='screen',
                parameters=[{
                    'n_agents': N,
                    }]))
            

    # add executables for each robot
    for i in range(N):
        initial_position = P[i, :].tolist()
        goals_i = [item for goal in goals[i] for item in goal]

        # webots exec
        launch_description.append(get_webots_driver_cf(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':f'agent_{i}'},
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))
        
        # controller
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_my_formation_webots_controller', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'agent_id': i,
                }]))

        # guidance
        launch_description.append(Node(
            package='crazychoir_examples', 
            executable='crazychoir_my_formation_webots_guidance', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'agent_id': i,
                'N': N,
                'init_pos': initial_position,
                'goals': goals_i,
                'ksi': ksi_flatten,
                'a_matrix': a_matrix_flat,
                }]))

        # reference
        launch_description.append(Node(
            package='crazychoir_examples',
            executable='crazychoir_my_formation_webots_trajectory', 
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{     
                'agent_id': i, 
                }]
                

    return LaunchDescription(launch_description)
