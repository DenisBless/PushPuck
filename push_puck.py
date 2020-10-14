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

        self.robot_init_qpos = np.array([4.96976216e-05, - 1.84996010e-01, - 4.63546468e-05, - 2.08528506e+00,
                                         -8.95942123e-06, 1.90028276e+00, 7.85404067e-01, 0., 0.])

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
