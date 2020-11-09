from push_puck.push_puck_robot import PushPuck3DoF, PushPuck7DoF
from push_puck.push_puck_no_robot import PushPuckNoRobot
from push_puck.push_puck_mp import PushPuckMultiProcessing
import matplotlib.pyplot as plt
import numpy as np
import time

if __name__ == '__main__':
    DoF = 7
    NSAMPLES = 100
    NWORKER = [1, 2, 6, 8, 10]

    times = []

    for nworker in NWORKER:
        start_time = time.time()
        weights = np.random.randn(NSAMPLES, DoF * 5)
        pp_mp = PushPuckMultiProcessing(push_puck=PushPuck7DoF, n_workers=nworker, render=False)
        target_pos = np.random.randn(NSAMPLES, 3)

        rewards = pp_mp(weights=weights, goal_pos=target_pos)
        time_taken = time.time() - start_time
        times.append(time_taken)
        print("Time for", NSAMPLES, "samples with", str(nworker), "worker: ", time_taken)

    plt.plot(NWORKER, times)
    plt.xlabel("number of worker")
    plt.ylabel("time")
    plt.show()

