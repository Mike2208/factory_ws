#!/usr/bin/env python

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # suppress warnings
import numpy as np
import matplotlib.pyplot as plt

from working_memory.ros_working_memory import WorkingMemory


if __name__ == "__main__":
    ### test behavior of the two memory modules
    wm = WorkingMemory(wm_type='gru')

    act, inp = [], []
    for i in range(20):  # 20 is max timesteps for workmate
        if i == 4:
            wm.input(1)
        elif i == 7:
            wm.input(-1)
        elif i == 11:
            wm.input(1)
        elif i == 14:
            wm.input(-1)
        else:
            wm.input(0)
        inp.append(wm.o)
        act.append(wm.output())

    t = wm.get_working_memory_time()
    tspan = np.arange(0, t, wm.dt)

    plt.figure()
    plt.subplot(211)
    plt.plot(tspan, inp), plt.grid()
    plt.yticks([-1, 0, 1], ['nh', 'ns', 'h'])  # Signals: No Human, No trigger Signal, Human
    plt.ylabel('observation [-]')

    plt.subplot(212)
    plt.plot(tspan, act), plt.grid()
    plt.xlabel('time [s]'), plt.yticks([-1, 1], ['fast', 'slow'])  # responses
    plt.ylabel('chosen action [-]')
    plt.show()
