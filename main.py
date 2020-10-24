from push_puck_no_robot import PushPuckNoRobot


# Test to see the puck moving without robot model
if __name__ == '__main__':
    pp = PushPuckNoRobot()
    pp.rollout(weights=[25, 0], extra_timesteps=5000)