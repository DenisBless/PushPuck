import numpy as np
from pathlib import Path
from mujoco_py import MjSim, MjViewer, load_model_from_path, MjSimState
from push_puck.utils.helper import set_puck
from abc import abstractmethod, ABC


class PushPuckBase(ABC):
    def __init__(self,
                 n_substeps: int = 1,
                 render: bool = False):
        self.render = render

        set_puck(raw_xml_path=self.raw_xml_path, xml_path=self.xml_path, puck_size=None, puck_pos=None)
        model = load_model_from_path(self.xml_path)

        self.sim = MjSim(model=model, nsubsteps=n_substeps)
        self.viewer = MjViewer(self.sim) if render else None

        self.reset()

    def __call__(self, weights, extra_timesteps=1000):
        self.reset()
        return self.rollout(weights, extra_timesteps)

    def reset(self) -> None:
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
    def xml_path(self) -> str:
        return str(Path(__file__).resolve().parents[0]) + '/' + 'assets/xml_model/env_model.xml'

    def set_target(self, target_pos) -> None:
        """
        Sets the postion of the target.
        :param target_pos: Desired target position
        """
        if target_pos is None:
            target_pos = [0.7, 0, 0.02]

        body_id = self.sim.model.body_name2id('target')
        self.sim.model.body_pos[body_id] = target_pos
        self.sim.forward()

    @property
    def target_pos(self):
        """
        Helper for getting the current target position.
        :return: Current target position
        """
        return self.sim.data.get_site_xpos('target:site1').copy()

    @property
    def puck_pos(self):
        """
        Helper for getting the current puck position.
        :return: Current puck position
        """
        return self.sim.data.get_body_xpos('puck').copy()

    @property
    def tcp_pos(self):
        """
        Helper for getting the current end effector position.
        :return: Current end effector position
        """
        return self.sim.data.get_body_xpos('tcp').copy()

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
    def rollout(self, weights, goal_pos, extra_timesteps=200):
        raise NotImplementedError
