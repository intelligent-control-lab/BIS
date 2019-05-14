import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy.random import rand, randn
from numpy.linalg import norm, inv
from numpy import cos, sin, arccos, sqrt, pi, arctan2, cross, dot

def is_between(x, p1, p2):
    return dot(x - p1, p2 - p1) > 0 and dot(x - p2, p1 - p2) > 0

def point_2_seg(x, p1, p2):
    d1 = norm(x - p1)
    d2 = norm(x - p2)
    l = norm(p1 - p2)
    if not is_between(x, p1, p2):
        if d1 < d2:
            return [d1, 0]
        else:
            return [d2, l]
    else:
       
        dis = abs(cross(p1-x, p2-x)) / l;
        lm = dot(x - p1, p2 - p1) / norm(p2 - p1)
       
        return [dis, lm]
        
x = np.array([5,5])
p1 = np.array([1,2])
p2 = np.array([4,4])
print(point_2_seg(x, p1, p2))