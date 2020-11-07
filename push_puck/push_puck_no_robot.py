import numpy as np
from pathlib import Path
from push_puck.push_puck_base import PushPuckBase
from push_puck.utils.helper import set_puck, set_target


class PushPuckNoRobot(PushPuckBase):
    def __init__(self,
                 nsubsteps: int = 1,
                 render: bool = True):
        super().__init__(nsubsteps=nsubsteps, render=render)

    @property
    def robot_init_qpos(self):
        return np.array([0, 0])

    @property
    def robot_final_qpos(self):
        return np.array([0, 0])  # Todo

    @property
    def raw_xml_path(self):
        return str(Path(__file__).resolve().parents[0]) + '/assets/xml_model/env_model_no_robot_raw.xml'

    def rollout(self, weights, target_pos=None, extra_timesteps=200):
        set_target(raw_xml_path=self.raw_xml_path, xml_path=self.xml_path, target_pos=target_pos)
        self.sim.reset()
        self.sim.data.ctrl[:] = weights
        self.sim.step()
        if self.render:
            self.viewer.render()
        for _ in range(extra_timesteps):
            self.sim.data.ctrl[:] = np.zeros_like(weights)
            self.sim.step()
            if self.render:
                self.viewer.render()
