import numpy as np
from pathlib import Path
from push_puck_base import PushPuckBase


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

    def rollout(self, weights):
        for i in range(10000):
            self.viewer.render()
