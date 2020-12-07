import numpy

from Parallel_Hybrid_Model import *

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
