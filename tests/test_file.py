from push_puck.push_puck_robot import PushPuck7DoF
import numpy as np
from multiprocessing import Process

# Test to see the puck moving without robot model
if __name__ == '__main__':
    # weights = [25, 0]
    # pp = PushPuckNoRobot()

    weights = np.random.randn(15)
    pp = PushPuck7DoF(render=False)
    reward = pp.rollout(weights=weights, extra_timesteps=1000)

    print(reward)