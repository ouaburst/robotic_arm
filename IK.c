import numpy as np
import math

h = 23.5
L = 5
M = 8.5
N = 17

x = 29.84
y = 0
z = 3.9 


R = math.sqrt(pow(x,2) + pow(y,2))
S = R - N
Q = math.sqrt(pow(S,2) + pow(z,2))
f = np.arctan2(z, S)
g = np.arccos((pow(L,2) + pow(Q,2)-pow(M,2))/(2*L*Q))
a = f + g
b = np.arccos((pow(M,2) + pow(L,2)-pow(Q,2))/(2*L*M))
c = -b-a+2*np.pi

print ("x: ",x) 
print ("y: ",y)
print ("z: ",z)
print ("================")
print ("R: ",R) 
print ("S: ",S)
print ("Q: ",Q)
print ("f: ",np.rad2deg(f))
print ("g: ",np.rad2deg(g))
print ("-> a: ",np.rad2deg(a))
print ("-> b: ",np.rad2deg(b))
print ("-> c: ",np.rad2deg(c))


