from simsim import testsimsim
from math import pi  as pi
######################################################
#parametres du test
Te = 0.050 #s
Rroue = 0.195/2 #m
L = 0.57 #m
phi_1st_half = 40 #degrees
phi_2nd_half = 10 #degrees
alpha_1st_half = 0 #radians
alpha_2nd_half = pi/4 #radians
x0 = 0 #m
y0 = 0 #m
theta0 = pi/2  #radians
V0 = 0  #m/s
#x = 0, y = 0, theta = 0, v = 0
nb_steps_1st_half = 12
nb_steps_2nd_half = 28

from testsimsim import testsimsim
from math import pi  as pi
######################################################
#parametres du test
Te = 0.050 #s
Rroue = 0.195/2 #m
L = 0.57 #m
phi_1st_half = 40 #degrees
phi_2nd_half = 10 #degrees
alpha_1st_half = 0 #radians
alpha_2nd_half = -pi/3
x0 = 0 #m
y0 = 0 #m
theta0 = pi/2  #radians
V0 = 0  #m/s
#x = 0, y = 0, theta = 0, v = 0
nb_steps_1st_half = 12
nb_steps_2nd_half = 28



print("Forward then on the right forward")
testsimsim(phi_1st_half, phi_2nd_half, alpha_1st_half, alpha_2nd_half, x0, y0, theta0, V0, Rroue, L, Te, nb_steps_1st_half, nb_steps_2nd_half)
