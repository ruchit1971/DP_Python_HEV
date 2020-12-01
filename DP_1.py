
# Import Libraries
import tkinter as tk
from tkinter.ttk import *
import numpy
from tkinter import BOTH
import matplotlib.pyplot as plt
from scipy.io import loadmat
from PIL import ImageTk, Image

global data
data = loadmat('City_MAN_DDP.mat')

# Velocity...
#global V_z
V_z = data['V_z']
G_z = V_z.astype(float)



# Gear Grid
G_z = data['G_z']
G_z = G_z.astype(float)


# State of Charge grid
Initial_SOC = float(0.5)
length = len(V_z)
Time = range(length)
T_z = data['T_z']
SOC_grid = numpy.arange(0.4, 0.61, 0.001)
w = (numpy.abs(SOC_grid-Initial_SOC)).argmin()
# Final Cost Grid
finalCost = numpy.zeros(len(SOC_grid))
finalCost[numpy.where(SOC_grid < 0.5)] = numpy.inf



# Function file for paralleHybrid to calculate the stepcost.

def parallelHybrid(t_vec, SOC_start, SOC_final, G_z, V_z):
    # Inputs:
    # t_vec:- 1x2 matrix, with the start and stop time for the interval.
    # SOC_start:- A single start value for SO during the interval.
    # SOC_final:- Vector (from the dicretization) with all possible final values for the interval.

    # Outputs:
    # costVector - Vector with the costs for all arcs from SOC_start.

    # Vehicle Parameters:-
    # Lower Heating Value
    H_l = 44.6e6  # J/kg

    # Fuel Density
    roh_l = 732.2  # Kg/m3

    # Air Density
    roh_a = 1.18  # kg/m3

    # Engine Inertia
    Je = 0.2  # kgm2

    # Engine Maximum Torque
    T_engine_max = 115  # Nm

    # Engine Displacment
    V_disp = 1.497e-3  # m3

    # Willans approximation of engine efficiency and pressure
    e = 0.4
    p_me0 = 0.1e6  # MPa

    # Battery charging capacity
    Q_o = 6.5  # Ah

    # Open circuit voltage
    Uoc = 300  # V

    # Maximum dis-/charging current
    Imax = 200  # A
    Imin = -200  # A

    # Inner resistance
    Ri = 0.65  # ohm

    # Efficiency of electrical machine
    n_electricmachine = 0.9

    # Gravity
    g = 9.81

    # Drag coefficient
    cD = 0.32

    # Rolling resistance coefficient
    cR = 0.015

    # Frontal area
    Af = 2.31  # m2

    # Vehicle mass
    mv = 1500  # kg

    # Wheel radius
    rw = 0.3  # m

    # Inertia of the wheels
    Jw = 0.6  # kgm2
    mwheel = Jw / (rw ** 2)

    # Efficiency of Transmission
    eta_gearbox = 0.98

    # Electric machine Maximum Torque
    T_em_max = 400  # Nm

    # Power of Electric Machine
    P_em_max = 50  # kW

    # Electric motor weight
    m_em = 1.5  # kg/Kw

    # Maximum powertrain power
    P_pt_max = 90.8  # kW

    # Average speed at wheel.
    time1 = int(t_vec[0])
    time2 = int(t_vec[1])
    Average_speed_wheel = (float(V_z[time1]) + float(V_z[time2])) /2

    # Average acceleration at wheel
    Average_accl_wheel = float(V_z[time2] - V_z[time1])

    # Angular speed at wheel
    Angular_speed_wheel = Average_speed_wheel / rw

    # Angular acceleration at wheel
    Angular_acc_wheel = float(Average_accl_wheel) / rw

    # Gear Ratio
    G_z[G_z == 0] = 0
    G_z[G_z == 1] = 13.0529
    G_z[G_z == 2] = 8.1595
    G_z[G_z == 3] = 5.6651
    G_z[G_z == 4] = 4.2555
    G_z[G_z == 5] = 3.2623
    GearRatio = float(G_z[time1])

    # Traction force at Wheel
    Forcetraction = (0.5 * roh_a * Af * cD * (Average_speed_wheel ** 2)) + (mv * g * cR) + (
            (mv + mwheel) * Average_accl_wheel)

    # Torque at wheel
    Torquewheel = Forcetraction * rw

    # Torque of Gearbox
    if GearRatio == 0:
        Torquegearbox = 0
    else:
        Torquegearbox = Torquewheel / (float(GearRatio) * eta_gearbox ** numpy.sign(Torquewheel))

    # Angular Velocity at Engine
    Angular_speed_engine = Angular_speed_wheel * float(GearRatio)

    # Angular Acceleration at Engine
    Angular_accl_engine = Angular_acc_wheel * float(GearRatio)

    # Battery Dynamics.........

    # Charging / Dis-charging Current
    I_batt = -((SOC_final - SOC_start) * (Q_o * 3600))

    # Power of Battery
    PowerBattery = (Uoc * I_batt) - (Ri * (I_batt ** 2))

    # Torque of Electric Machine
    if GearRatio == 0:
        TorqueElectric_machine = (PowerBattery * (n_electricmachine ** numpy.sign(I_batt))) * 0
    else:
        TorqueElectric_machine = (PowerBattery * (n_electricmachine ** numpy.sign(I_batt))) / Angular_speed_engine

    # Torque of Engine
    Torqueengine = Torquegearbox - TorqueElectric_machine

    # Fuel consumption
    x = ((p_me0 * V_disp) / (4 * 3.14))
    costVector = (Angular_speed_engine / (e * H_l)) * (Torqueengine + x + Je * (Angular_accl_engine))

    costVector[numpy.where(Torqueengine > 115)] = numpy.inf
    costVector[numpy.logical_and(GearRatio == 0, I_batt == 0)] = 0
    costVector[numpy.logical_or(TorqueElectric_machine < -400, TorqueElectric_machine > 400)] = numpy.inf
    costVector[numpy.logical_and(abs(I_batt) > 0, GearRatio == 0)] = numpy.inf
    costVector[numpy.logical_or(I_batt < -200, I_batt > 200)] = numpy.inf
    costVector[numpy.where(costVector < 0)] = 0
    return costVector


# Dynamic Programming algorithm
def dynProg1D(time, SOC_grid, disc, G_z, V_z):
    last_element = len(time)
    next = numpy.zeros((len(time), len(SOC_grid)), dtype=float)
    costToGo = numpy.zeros((len(time), len(SOC_grid)), dtype=float)
    costToGo[last_element - 1, :] = costToGo[last_element - 1, :] + disc
    ele1 = numpy.arange(last_element - 1, 0, -1)
    ele2 = numpy.arange(0, len(disc), 1)

    # Outer loop over time.
    for idx1 in ele1:
        t1 = numpy.where(idx1 == time)
        t = int(t1[0])
        # Inner loops over the grid
        for idx2 in ele2:
            T_z = numpy.zeros(2, dtype=float)
            T_z[0] = t
            T_z[1] = t + 1
            cost = parallelHybrid(T_z, SOC_grid[idx2], SOC_grid, G_z, V_z)
            currCost = cost + costToGo[t + 1, :]
            val = numpy.min(currCost)
            ii = numpy.argmin(currCost)

            # Store the best arc
            next[t, idx2] = ii

            # Store the associated cost
            costToGo[t, idx2] = currCost[ii]

    return costToGo, next


value, SOC_path = dynProg1D(T_z, SOC_grid, finalCost, G_z, V_z)


# Graph.
last_element = len(T_z)
o = w
a = numpy.zeros(len(T_z),  dtype=float)
x = numpy.zeros(len(T_z),  dtype=float)
elex1 = numpy.arange(0, last_element - 1, 1)
SOC = numpy.zeros(len(T_z)-1,  dtype=float)

for idx1 in elex1:
    if SOC_path[idx1, int(o)] == o:
        a[idx1] = SOC_path[idx1, int(o)]
        rr = a[idx1]
    else:
        o = SOC_path[idx1, int(o)]
        a[idx1] = o
        rr = a[idx1]
    SOC[idx1] = SOC_grid[int(rr)]
    ttr = len(SOC)
    #SOC[ttr] = SOC_grid[int(o)]

plt.plot(SOC)
plt.ylabel('State of Charge')
plt.show()



print(value)
