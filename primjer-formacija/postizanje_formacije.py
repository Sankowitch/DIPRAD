import numpy as np
import matplotlib.pyplot as plt
import time

"""
Jednostavan primjer za postizanje
jedne formacije za 3 agenta
za 2.2. poglavlje
#za poglavlja gdje se pokazuju samo fomacije control_gain = 0.5
#za poglavlja gdje se pokazuje izbjegavanje sudara control gain= 0.1
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

def calculate_dx(a, x, e,  control_gain=0.1):
    dx =  np.zeros_like(x)
    
    n = len(x)
    for i in range(n):
        dx_elem = [0, 0]
        for j in range(n):
            dx_elem += a[i][j] * ((x[j] - x[i]) - (e[j] - e[i]))
            
        dx[i] = dx_elem * control_gain
        
    return dx


#calculate dx uz nesudaranje
#dodano da se prije ne trose resursi za racunanje separacije 
def calculate_dx_sep(a, x, e,  control_gain=0.1, c = 0.2):
    dx =  np.zeros_like(x)
    
    n = len(x)
    for i in range(n):
        dx_elem = [0, 0]
        sep_element = [0, 0]
        for j in range(n):
            if i == j:
                continue
            dx_elem += a[i][j] * ((x[j] - x[i]) - (e[j] - e[i]))
            norm = (x[j][0] - x[i][0])**2 + (x[j][1] - x[i][1])**2
            sep_element += (x[j] - x[i]) / norm
            
        dx[i] = dx_elem * control_gain - sep_element * c
        
    return dx


def consensus_protocol(x, y, x_arr, e_arr, adjacency_matrix , graph, separation = True):
    
    

    nasli_se = False

    iter = 0
    dt = 0.1
    control_gain = 0.2

    

    x_list = []
    time_start = time.time()
    while True:
        iter = iter + 1

        if separation == True:
            dx = calculate_dx_sep(adjacency_matrix, x_arr, e_arr, control_gain)
        else:
            dx = calculate_dx(adjacency_matrix, x_arr, e_arr, control_gain)
        x_arr = update_x(x_arr, dx, dt)
        
        x_list.append(x_arr)

        if iter == 55:
            break
        
  
    time_end = time.time()
    time_delta = time_end - time_start
    print("Trajanje algortima:")
    print(time_delta)
    
    moving_plot(x, y, x_list, graph)


def choose_choreo():
    choreo = {"1", "2"}
    sudar = {"0", "1"}
    chosen_sudar = "0"

    print("IZABERI KOREOGRAFIJU 1 ILI 2 iz primjera")
    while True:
        chosen_choreo = input("Koreografija: ").strip().lower() 
        if chosen_choreo in choreo:
            if chosen_choreo == "2":
                print("Sa sudarom (0) ili bez (1)")
                while True:
                    chosen_sudar = input("Sudar: ").strip().lower()
                    if chosen_sudar not in sudar:
                        print("Krivi format upisa :(, probaj opet")
                        continue
                    break


            
            return chosen_choreo, chosen_sudar
        
        else:
            print("Krivi format upisa :(, probaj opet")


def change_to_arr(x, y):
    x_arr = np.array([[x[i], y[i]] for i in range(len(x))])
    x_arr = x_arr.astype(float)
    return x_arr


if __name__ == "__main__":


    chosen_choreo, sudar = choose_choreo()

    choreos = {}

    #KOREOGRAFIJA 1
    #početne poozicije - postavljene u liniju
    choreo_1 =  "koreografija 1"
    x_1 = [1, 2.5, 4]
    y_1 = [2.5, 2.5, 2.5]
    x_arr_1 = change_to_arr(x_1, y_1)

    #pozicije u prosotru - zelimo trokut koji gleda prema gore
    e_x_1 = [0 ,2, 4]
    e_y_1 = [0, 3, 0]
    e_arr_1 = change_to_arr(e_x_1, e_y_1)

    choreos["1"] = [choreo_1, x_arr_1, e_arr_1]

    #KOREOGRAFIJA 2
    choreo_2 = "koreografija 2"
    #pocetne pozicije iste kao i kod prve koreografije
    x_arr_2 = x_arr_1
    #pozicije u prosotru - zelimo trokut koji gleda prema gore
    e_x_2 = [4 ,0, 2]
    e_y_2 = [0, 0, 3]
    e_arr_2 = change_to_arr(e_x_2, e_y_2)

    choreos["2"] = [choreo_2, x_arr_2, e_arr_2]

    separation = False
    if sudar == "1":
        separation = True


    choreo = choreos[chosen_choreo][0]
    x_arr =  choreos[chosen_choreo][1]
    e_arr =  choreos[chosen_choreo][2]






    adjacency_matrix = np.array([[0, 1, 1, 1], [1, 0, 1, 1], [1, 1, 0, 1], [1, 1, 1, 0]])
    



    n = len(x_arr)
    
 
    


    print(("MATRICA SUSJEDSTVA ZA {}:\n").format(choreo))
    print(adjacency_matrix)
    print("\n-----------------------")
    consensus_protocol(x_1, y_1, x_arr, e_arr, adjacency_matrix, choreo, separation)