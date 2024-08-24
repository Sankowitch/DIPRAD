
import numpy as np
import matplotlib.pyplot as plt

""""
RADJENO SA 2 DIMENZIJE
kod za randevu algoritam koji ukljucuje 
grafove sa 4 agenta iz poglavlja 1.2 i 1.3
"""

def moving_plot(x, y, r_array, graph):
    plt.figure(num=graph)

    labels = ["1", "2", "3", "4"] 
    colors = ["blue", "green", "orange", "purple"] 

    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.gca().set_aspect('equal', adjustable='box')

    # Add labels to the initial positions
    for i in range(len(labels)):
        plt.text(x[i], y[i], labels[i], fontsize=12, ha='right', va='bottom')

    # Plot each frame with its respective colors
    for t, r in enumerate(r_array):
        for i in range(len(x)):
            plt.scatter(r[i, 0], r[i, 1], c=colors[i], label=labels[i] if t == 0 else "")
        plt.pause(0.1)

    plt.legend()
    plt.show()


def update_r(r, dr, dt):
    #dr -> brzina v
    #v = s/t  -> s = v*t = dr * dt
    #dt je step size
    #a dr je - aij * (rj - ri)
    r = r + dr * dt
    return r

def calculate_dr(a, r, control_gain=0.1):
    dr =  np.zeros_like(r)
    
    n = len(r)
    for i in range(n):
        dr_elem = [0, 0]
        for j in range(n):
            dr_elem += a[i][j] * (r[j] - r[i])
            
        dr[i] = dr_elem * control_gain
        
    return dr


#gleda kad je uvijet randevua zadovoljen - kada su dovoljno blizu
#dodano u kod jer se sve na kraju plota
#i zato što je ovo kod za jednostavnu vizualizaciju
def uvijet_prekidanja(a, r, tresh = 0.1):
    n = len(r)
    for i in range(n):
        
        for j in range(n):
            if a[i][j] == 0:
                continue
            udaljenost = np.linalg.norm(r[j] - r[i]) 
            if udaljenost >= tresh:
                return False
    
    
    return True


def consensus_protocol(x, y, r, adjacency_matrix , graph):
    n = len(x)
    

    nasli_se = False

    iter = 0
    dt = 0.1
    control_gain = 0.5

    

    r_list = [r]
    while nasli_se == False:
        iter = iter + 1

        dr = calculate_dr(adjacency_matrix, r, control_gain)
        r = update_r(r, dr, dt)
        
        r_list.append(r)
        
        
        nasli_se = uvijet_prekidanja(adjacency_matrix, r)
        #dodano jer neke koreografije ne konvergiraju

        if iter == 100 and nasli_se == False:
            break
        


    print(len(r_list))
    moving_plot(x, y, r_list, graph)


def choose_graph():
    graphs = {"a", "b", "c", "d", "e", "f"}

    print("IZABERI GRAF a, b, c, d, e ili f iz primjera")
    while True:
        chosen_graph = input("Graf: ").strip().lower() 
        if chosen_graph in graphs:
            
            return chosen_graph
        
        else:
            print("Krivi format upisa :(, probaj opet")



if __name__ == "__main__":
    
    #početne poozicije - postavljeni u kvadrat (kao iz primjera)
    x = [1, 4, 4, 1]
    y = [4, 4, 1, 1]

    #states, (stanja svih agenata ubacena su u matricu r)
    r = np.array([[x[i], y[i]] for i in range(len(x))])
    r = r.astype(float)
     
    #broj agenata
    n = len(x)


    grafovi = {}
    #SA SLIKE 1

    #MATRICA SUSJEDSTVA ZA NEUSMJERENI GRAF (a)
    graph_a = "GRAF (a)"
    adjacency_matrix_a = np.array([[0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0]])
    grafovi["a"] = [graph_a, adjacency_matrix_a]

    #MATRICA SUSJEDSTVA ZA USMJERENI GRAF (b)
    graph_b = "GRAF (b)"
    adjacency_matrix_b = np.array([[0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
    grafovi["b"] = [graph_b, adjacency_matrix_b]

    #SA SLIKE 2

    #MATRICA SUSJEDSTVA ZA NEUSMJERENI GRAF (c)
    graph_c = "GRAF (c)"
    adjacency_matrix_c = np.array([[0, 1, 1, 0], [1, 0, 0, 0], [1, 0, 0, 1], [0, 0, 1, 0]])
    grafovi["c"] = [graph_c, adjacency_matrix_c]

    #MATRICA SUSJEDSTVA ZA NEUSMJERENI GRAF (d)
    graph_d = "GRAF (d)"
    adjacency_matrix_d = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0]])
    grafovi["d"] = [graph_d, adjacency_matrix_d]
    
    #SA SLIKE 3

    #MATRICA SUSJEDSTVA ZA USMJERENI GRAF (e)
    graph_e = "GRAF (e)"
    adjacency_matrix_e = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 1, 0]])
    grafovi["e"] = [graph_e, adjacency_matrix_e]

    #MATRICA SUSJEDSTVA ZA USMJERENI GRAF (f)
    graph_f = "GRAF (f)"
    adjacency_matrix_f = np.array([[0, 0, 0, 0], [1, 0, 1, 0], [0, 0, 0, 0], [1, 0, 1, 0]])
    grafovi["f"] = [graph_f, adjacency_matrix_f]


    
    izabrani_graf = choose_graph()

    graph = grafovi[izabrani_graf][0]
    adjacency_matrix = grafovi[izabrani_graf][1]


    print(("MATRICA SUSJEDSTVA ZA {}:\n").format(graph))
    print(adjacency_matrix)
    print("\n-----------------------")
    consensus_protocol(x, y, r, adjacency_matrix, graph)