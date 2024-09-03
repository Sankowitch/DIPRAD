






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