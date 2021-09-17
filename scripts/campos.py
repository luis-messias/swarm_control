import numpy as np
from numpy.core.numeric import Inf, nan
import math

def campo_atracao( x, y, x0_ , y0_ , beta = 1, r = 0.30, s = 0.80):
        angle = np.arctan2(x0_ - x, y0_ - y)
        d = math.sqrt((x-x0_)**2 + (y-y0_)**2)
        if d < r:
            U = 0
            V = 0

        elif d <= (s+r):
            U = beta*(d-r)*np.cos(angle)
            V = beta*(d-r)*np.sin(angle)

        else:
            U = beta*s*np.cos(angle)
            V = beta*s*np.sin(angle)

        return U, V

def campo_repulsao(list_x,list_y ,x, y, beta =  1, r = 0.001, s = 1):
        U = 0
        V = 0
        for i in range(len(list_x)):
            d = math.sqrt((x-list_x[i])**2 + (y-list_y[i])**2)
            angle = np.arctan2(list_x[i] - x, list_y[i] - y)
            if d < r:
                U += 0
                V += 0

            elif d <= (s+r):
                U += -beta*(s+r-d)*np.cos(angle)
                V += -beta*(s+r-d)*np.sin(angle)

            else:
                U += 0
                V += 0

        return U, V

def campo_repulsao2(list_x, list_y, x, y, beta = 2, r = 0.001, s = 0.5):        
        d_ = ((list_x - x)**2 + (list_y - y)**2)**0.5
        d_ = np.where(d_ > 0.00000001, d_, Inf)
        d = min(d_)
        indice = np.where(d_ == d)[0][0]

        U, V = 0, 0    

        if d < r:
            U += 0
            V += 0

        elif d <= (s+r):
            U += -beta*(s+r-d) *np.cos(np.arctan2(list_x[indice] - x, list_y[indice] - y))
            V += -beta*(s+r-d) *np.sin(np.arctan2(list_x[indice] - x, list_y[indice] - y))
        else:
            U += 0                
            V += 0

        return U, V