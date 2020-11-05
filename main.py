from push_puck_no_robot import PushPuckNoRobot
from push_puck_robot import PushPuck3DoF, PushPuck7DoF
import numpy as np
from multiprocessing import Process


def worker(weights, goal_pos, pid):
    pp = PushPuck7DoF()
    pp.rollout(weights, goal_pos)


if __name__ == '__main__':
    N_WORKERS = 6

    weights, goal_pos = ...  # StochasticSearchByMax()

    processes = [Process(target=worker, args=(weights, goal_pos, pid))
          for (weights, goal_pos, pid) in zip(weights, goal_pos, range(0, N_WORKERS))]

    for p in processes:
        p.start()

    for p in processes:
        p.join()

