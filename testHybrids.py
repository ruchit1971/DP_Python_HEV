import numpy
from tkinter import BOTH
import matplotlib.pyplot as plt
from scipy.io import loadmat
from PIL import ImageTk, Image
from dynProg1D import *


print('Enter Initial State of Charge [%] :-')
x = input()


global data
data = loadmat('City_MAN_DDP.mat')

# Velocity from Drive Cycle...
# global V_z
V_z = data['V_z']
V_z = V_z.astype(float)

# Gear Number from Drive cycle...
G_z = data['G_z']
G_z = G_z.astype(float)

# Create State of Charge grid...
Initial_SOC = float(x)/100
length = len(V_z)
Time = range(length)
T_z = data['T_z']
SOC_grid = numpy.arange(0.4, 0.61, 0.001)
w = (numpy.abs(SOC_grid - Initial_SOC)).argmin()


# Final Cost Grid (Final Constraint)...
finalCost = numpy.zeros(len(SOC_grid))
finalCost[numpy.where(SOC_grid < 0.5)] = numpy.inf


# Function file for Trapz.
def trapz(input, grid, dx):
    length = len(input)
    input1 = input[0:length - 1]
    time = numpy.arange(0, length - 1, 1)
    result = 0
    for idx in time:
        fx = ((input[idx] + input[idx + 1]) / 2) * dx
        result = result + float(fx)
    return result



# Calculate the Optimal Trajectory of State of Charge
value, SOC_path = dynProg1D(T_z, SOC_grid, finalCost, G_z, V_z)

# Final State of Charge Graph...
last_element = len(T_z)
o = w
a = numpy.zeros(len(T_z), dtype=float)
x = numpy.zeros(len(T_z), dtype=float)
elex1 = numpy.arange(0, last_element - 1, 1)
SOC = numpy.zeros((len(T_z) ,1), dtype=float)
FC = numpy.zeros(len(T_z) - 1, dtype=float)

for idx1 in elex1:
    if SOC_path[idx1, int(o)] == o:
        a[idx1] = SOC_path[idx1, int(o)]
        rr = a[idx1]
        x[idx1] = value[idx1, int(o)]
        ee = x[idx1]
    else:
        o = SOC_path[idx1, int(o)]
        a[idx1] = o
        rr = a[idx1]
        x[idx1] = value[idx1, int(o)]
        ee = x[idx1]
    SOC[idx1] = SOC_grid[int(rr)]
    FC[idx1] = ee
    ttr = len(SOC)

SOC[last_element-1] = SOC_grid[w]
x_tot = trapz(V_z, T_z, 1)
Fuel_consumption = ((FC[0] / 0.7372) * 100) / (x_tot / 1000)

# Fuel Consumption...
print('Results')
print('Fuel consumption:-', Fuel_consumption, 'l/100km')

plt.plot(T_z, SOC*100, linewidth=2.0)
plt.ylabel('State of Charge')
plt.xlabel('Time')
plt.show()

