#!/usr/bin/env python3

import numpy as np

def TL2B(q):
    return np.array([[2*(q[0]^2+q[1]^2)-1,2*(q[1]*q[2]-q[0]*q[3]),2*(q[1]*q[3]+q[0]*q[2])],
                     [2*(q[1]*q[2]+q[0]*q[3]),2*(q[0]^2+q[2]^2)-1,2*(q[2]*q[3]-q[0]*q[1])],
                     [2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),2*(q[0]^2+q[3]^2)-1]])

def TB2L(q):
    return np.transpose(TL2B(q))

def norm(v,n=2):
    return np.power(np.sum(np.power(v,n)),1./n)

def normalize(v):
    vmag = np.linalg.norm(v,2)
    if vmag == 0:
        return v
    else:
        return v/vmag
    
def clip(val,min,max):
    return np.minimum(max,np.maximum(min,val))
    