import numpy as np
import matplotlib.pyplot as plt
import time

"""
Jednostavan primjer za postizanje
jedne formacije za 3 agenta
za 2.4. poglavlje
"""



def moving_plot(x, y, x_array, graph):
    plt.figure(num=graph)

    labels = ["1", "2", "3"] 
    colors = ["blue", "purple", "green"] 

    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.gca().set_aspect('equal', adjustable='box')

    # Add labels to the initial positions
    for i in range(len(labels)):
        plt.text(x[i], y[i], labels[i], fontsize=12, ha='right', va='bottom')
        plt.scatter(x[i], y[i], color = "black")

    # Plot each frame with its respective colors
    for t, r in enumerate(x_array):
        for i in range(len(x)):
            plt.scatter(r[i, 0], r[i, 1], c=colors[i], label=labels[i] if t == 0 else "")
        plt.pause(0.1)




    # Plot the last points in black
    last_positions = x_array[-1]
    for i in range(len(x)):
        plt.scatter(last_positions[i, 0], last_positions[i, 1], c='red')

    plt.legend()
    plt.show()


def update_x(x, dx, dt):
    #dr -> brzina v
    #v = s/t  -> s = v*t = dr * dt
    #dt je step size
    #a dr je - aij * (rj - ri)
    x = x + dx * dt
    return x

def calculate_dx(a, x, e, g,  control_gain=0.1, leader_index = 0):
    dx =  np.zeros_like(x)
    
    n = len(x)
    for i in range(n):
        if i == leader_index:
            continue
        dx_elem = [0, 0]
        for j in range(n):
            dx_elem += a[i][j] * ((x[j] - x[i]) - (e[j] - e[i]))
            
        dx[i] = dx_elem * control_gain + g[i] * (x[0] - x[i])
        
    return dx


def consensus_protocol(x, y, x_arr, e_arr, adjacency_matrix , graph, g):
    
    iter = 0
    dt = 0.1
    control_gain = 0.2

    v = np.array([0.7, 0.7]) 

    x_list = []
    time_start = time.time()
    while True:
        iter = iter + 1

       
        
        dx = calculate_dx(adjacency_matrix, x_arr, e_arr, g ,control_gain)
        x_arr = update_x(x_arr, dx, dt)
        
        #updejt leadera
        x_arr[0] += v * dt
        
        x_list.append(x_arr)

        if iter == 35:
            break
        
  
    time_end = time.time()
    time_delta = time_end - time_start
    print("Trajanje algortima:")
    print(time_delta)
    
    moving_plot(x, y, x_list, graph)





def change_to_arr(x, y):
    x_arr = np.array([[x[i], y[i]] for i in range(len(x))])
    x_arr = x_arr.astype(float)
    return x_arr


if __name__ == "__main__":

    choreos = {}

    #KOREOGRAFIJA 3 (jer izgleda drukacije od onih prije)
    #početne poozicije - postavljene u liniju
    choreo =  "koreografija 3"
    x_1 = [1, 2, 3]
    y_1 = [1, 1, 1]
    x_arr = change_to_arr(x_1, y_1)

    #pozicije u prosotru - zelimo trokut koji gleda prema gore
    e_x_1 = [1.5 ,0, 3]
    e_y_1 = [1.5, 0,  0]
    e_arr = change_to_arr(e_x_1, e_y_1)

    
    adjacency_matrix = np.array([[0, 1, 1, 1], [1, 0, 1, 1], [1, 1, 0, 1], [1, 1, 1, 0]])
    g = np.array([0, 1, 1])   #pinning gains



    n = len(x_arr)
    


    print(("MATRICA SUSJEDSTVA ZA {}:\n").format(choreo))
    print(adjacency_matrix)
    print("\n-----------------------")
    consensus_protocol(x_1, y_1, x_arr, e_arr, adjacency_matrix, choreo, g)