from push_puck_no_robot import PushPuckNoRobot
from push_puck_robot import PushPuck3DoF, PushPuck7DoF
import numpy as np


# Test to see the puck moving without robot model
if __name__ == '__main__':
    # weights = [25, 0]
    # pp = PushPuckNoRobot()

    weights = np.random.randn(15)
    pp = PushPuck7DoF()
    pp.rollout(weights=weights, extra_timesteps=5000)
