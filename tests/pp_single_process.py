from push_puck.push_puck_robot import PushPuck3DoF, PushPuck7DoF
from push_puck.push_puck_no_robot import PushPuckNoRobot
import numpy as np

if __name__ == '__main__':
    DoF = 3

    if DoF == 2:
        weights = np.random.randn(DoF)
        pp = PushPuckNoRobot(render=True)

    elif DoF == 3:
        weights = np.random.randn(DoF * 5)
        pp = PushPuck3DoF(render=True)

    elif DoF == 7:
        weights = np.random.randn(DoF * 5)
        pp = PushPuck7DoF(render=True)

    else:
        raise ValueError("Invalid DoF.")

    reward = pp.rollout(weights=weights, extra_timesteps=1000)
    print("Reward: ", reward)
