from pathlib import Path
import numpy as np
from mujoco_py import MjSim, MjViewer, load_model_from_path, MjSimState


class PushPuck:
    def __init__(self,
                 nsubsteps=1,
                 render=True):
        xml_path = str(Path(__file__).resolve().parents[0]) + '/env_model.xml'
        model = load_model_from_path(xml_path)

        self.sim = MjSim(model=model, nsubsteps=nsubsteps)
        self.viewer = MjViewer(self.sim) if render else None

        self.robot_init_qpos = np.array([0, 0.202, 0, -2.86, 0, 1.98, 0.771, 0, 0])

        self.reset()

    def reset(self):
        """Resets the environment (including the agent) to the initial conditions.
        """
        self.sim.reset()
        # Set initial position and velocity
        self.qpos = self.sim.data.qpos.copy()
        self.qpos[:self.robot_init_qpos.shape[0]] = self.robot_init_qpos
        qvel = np.zeros(self.sim.data.qvel.shape)
        mjSimState = MjSimState(time=0.0, qpos=self.qpos, qvel=qvel, act=None, udd_state={})
        self.sim.set_state(mjSimState)
        self.sim.forward()


if __name__ == '__main__':
    pp = PushPuck()
    # Only make joint 2,4 and 6 controllable!

    for i in range(10000):
        pp.viewer.render()
