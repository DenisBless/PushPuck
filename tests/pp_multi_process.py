from push_puck.push_puck_robot import PushPuck3DoF, PushPuck7DoF
from push_puck.push_puck_no_robot import PushPuckNoRobot
from push_puck.push_puck_mp import PushPuckMultiProcessing
import numpy as np
from multiprocessing import Process

# Test to see the puck moving without robot model
if __name__ == '__main__':
    DoF = 3
    NWORKER = 4
    NSAMPLES = 32

    if DoF == 2:
        weights = np.random.randn(NSAMPLES, DoF)
        pp_mp = PushPuckMultiProcessing(push_puck=PushPuckNoRobot, n_workers=NWORKER, render=False)

    elif DoF == 3:
        weights = np.random.randn(15)
        pp_mp = PushPuckMultiProcessing(push_puck=PushPuck3DoF, n_workers=NWORKER, render=False)

    elif DoF == 7:
        weights = np.random.randn(35)
        pp = PushPuck7DoF(render=True)

    else:
        raise ValueError("Invalid DoF.")

    target_pos = np.random.randn(NSAMPLES, 3)

    rewards = pp_mp(weights=weights, goal_pos=target_pos)
    print("Rewards: ", rewards)

