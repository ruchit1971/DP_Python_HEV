import numpy

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
    Average_speed_wheel = (float(V_z[time1]) + float(V_z[time2])) / 2

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