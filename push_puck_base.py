import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mujoco_py import MjSim, MjViewer, load_model_from_path, MjSimState
from utils.helper import set_puck
from abc import abstractmethod, ABC


class PushPuckBase(ABC):
    def __init__(self,
                 nsubsteps: int = 1,
                 render: bool = True):
        self.render = render

        set_puck(raw_xml_path=self.raw_xml_path, xml_path=self.xml_path, puck_size=None, puck_pos=None)
        model = load_model_from_path(self.xml_path)

        self.sim = MjSim(model=model, nsubsteps=nsubsteps)
        self.viewer = MjViewer(self.sim) if render else None

        # self.joint_indices = [x for x in range(1, num_dof + 1)]

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

    @property
    def xml_path(self):
        return str(Path(__file__).resolve().parents[0]) + '/' + 'assets/xml_model/env_model.xml'

    @property
    @abstractmethod
    def raw_xml_path(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def robot_init_qpos(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def robot_final_qpos(self):
        raise NotImplementedError

    @abstractmethod
    def rollout(self, weights):
        raise NotImplementedError
