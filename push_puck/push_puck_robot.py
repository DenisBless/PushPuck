import numpy as np
from pathlib import Path
from push_puck.push_puck_base import PushPuckBase
from mp_lib.dmps import DMP
from mp_lib.phase import ExpDecayPhaseGenerator
from mp_lib.basis import DMPBasisGenerator
import matplotlib.pyplot as plt


class PushPuckRobot(PushPuckBase):
    def __init__(self,
                 n_substeps: int = 1,
                 render: bool = True):
        super().__init__(n_substeps=n_substeps, render=render)

    @property
    def raw_xml_path(self):
        return str(Path(__file__).resolve().parents[0]) + '/assets/xml_model/env_model_robot_raw.xml'

    @property
    def robot_init_qpos(self):
        return np.array([0, 0.202, 0, -2.86, 0, 1.98, 0.771, 0, 0])

    @property
    def robot_final_qpos(self):
        return np.array([0, 1.2, 0, -2.86, 0, 4.1, 0.771, 0, 0])


class PushPuck3DoF(PushPuckRobot):
    def __init__(self,
                 n_substeps: int = 1,
                 render: bool = True):
        super().__init__(n_substeps=n_substeps, render=render)

    def rollout(self, weights, target_pos=None, extra_timesteps=200):
        self.set_target(target_pos=target_pos)
        weights = np.reshape(weights, (-1, 3))
        n_steps = weights.shape[0]

        dt = self.sim.model.opt.timestep * self.sim.nsubsteps
        num_basis = 9
        ep_len = 2  # in seconds
        n_time_steps = int(ep_len / dt)
        t = np.linspace(0, ep_len, n_time_steps)
        phase_generator = ExpDecayPhaseGenerator(duration=ep_len, alpha_phase=5)
        basis_generator = DMPBasisGenerator(phase_generator, num_basis=num_basis, duration=ep_len)

        dmp = DMP(num_dof=7,
                  basis_generator=basis_generator,
                  phase_generator=phase_generator,
                  num_time_steps=n_time_steps,
                  dt=dt)

        dmp.dmp_start_pos = self.robot_init_qpos[:-2].reshape((1, -1))

        weights = np.concatenate((np.zeros((n_steps, 1)),
                                  weights[:, 0][:, None],
                                  np.zeros((n_steps, 1)),
                                  weights[:, 1][:, None],
                                  np.zeros((n_steps, 1)),
                                  weights[:, 2][:, None],
                                  np.zeros((n_steps, 1))), axis=1)
        weights = np.concatenate((np.zeros((2, 7)), weights, np.zeros((2, 7))), axis=0)
        dmp.set_weights(weights,
                        goal=self.robot_final_qpos[:-2])
        des_pos, des_vel = dmp.reference_trajectory(t)

        plot_trajectory = True
        if plot_trajectory:
            plt.plot(des_pos)
            plt.pause(0.1)

        dists = []
        dists_final = []
        k = 0
        torques = []

        actual_pos = np.zeros_like(des_pos)
        error = False
        while k < des_pos.shape[0] + 200:
            # Compute the current distance from the ball to the inner part of the cup
            goal_pos = 0  # self.sim.data.site_xpos[goal_id]
            ball_pos = 0  # self.sim.data.body_xpos[ball_id]
            goal_final_pos = 0  # self.sim.data.site_xpos[goal_final_id]
            dists.append(np.linalg.norm(goal_pos - ball_pos))
            dists_final.append(np.linalg.norm(goal_final_pos - ball_pos))

            k_actual = np.minimum(des_pos.shape[0] - 1, k)

            cur_pos = self.sim.data.qpos[:7].copy()
            cur_vel = self.sim.data.qvel[:7].copy()
            actual_pos[k_actual, :] = cur_pos

            # Use MuJoCo's internal PD Controller
            gripper_ctrl = -1  # Keep gripper closed
            # Todo
            self.sim.data.ctrl[:] = np.concatenate((des_pos[k_actual], des_vel[k_actual], [gripper_ctrl, gripper_ctrl]))

            # Apply gravity compensation
            joint_indices = np.arange(1, 8)
            self.sim.data.qfrc_applied[joint_indices] = self.sim.data.qfrc_bias[joint_indices]

            # Forward the simulation
            self.sim.step()

            # Render the scene
            if self.render and self.viewer is not None:
                self.viewer.render()

            k += 1

        if plot_trajectory:
            plt.figure()
            plt.plot(actual_pos)
            plt.pause(0.1)

            # plt.figure()
            # plt.plot(np.vstack(torques))
            # plt.pause(0.1)
        min_dist = np.min(dists)
        return 0


class PushPuck7DoF(PushPuckRobot):
    def __init__(self,
                 n_substeps: int = 1,
                 render: bool = True):
        super().__init__(n_substeps=n_substeps, render=render)

    def rollout(self, weights, target_pos=None, extra_timesteps=200):
        self.set_target(target_pos=target_pos)
        weights = np.reshape(weights, (-1, 7))
        n_steps = weights.shape[0]

        dt = self.sim.model.opt.timestep * self.sim.nsubsteps
        num_basis = 9
        ep_len = 2  # in seconds
        n_time_steps = int(ep_len / dt)
        t = np.linspace(0, ep_len, n_time_steps)
        phase_generator = ExpDecayPhaseGenerator(duration=ep_len, alpha_phase=5)
        basis_generator = DMPBasisGenerator(phase_generator, num_basis=num_basis, duration=ep_len)

        dmp = DMP(num_dof=7,
                  basis_generator=basis_generator,
                  phase_generator=phase_generator,
                  num_time_steps=n_time_steps,
                  dt=dt)

        dmp.dmp_start_pos = self.robot_init_qpos[:-2].reshape((1, -1))

        weights = np.concatenate((weights[:, 0][:, None],
                                  weights[:, 1][:, None],
                                  weights[:, 2][:, None],
                                  weights[:, 3][:, None],
                                  weights[:, 4][:, None],
                                  weights[:, 5][:, None],
                                  weights[:, 6][:, None]
                                  ), axis=1)
        weights = np.concatenate((np.zeros((2, 7)), weights, np.zeros((2, 7))), axis=0)
        dmp.set_weights(weights,
                        goal=self.robot_final_qpos[:-2])
        des_pos, des_vel = dmp.reference_trajectory(t)

        plot_trajectory = False
        # if plot_trajectory:
        #     plt.plot(des_pos)
        #     plt.pause(0.1)

        dists = []
        dists_final = []
        puck_poss = []
        puck_vels = []
        k = 0
        torques = []

        puck_id = self.sim.model._body_name2id["puck"]
        goal_pos = np.array((1.15, 0, 0))  # self.sim.data.site_xpos[goal_id]

        actual_pos = np.zeros_like(des_pos)
        error = False
        while k < des_pos.shape[0] + extra_timesteps:
            # Compute the current distance from the ball to the inner part of the cup
            puck_pos = self.sim.data.body_xpos[puck_id].copy()
            puck_vel = self.sim.data.body_xvelp[puck_id].copy()
            puck_poss.append(puck_pos)
            puck_vels.append(puck_vel)
            goal_final_pos = 0  # self.sim.data.site_xpos[goal_final_id]
            dists.append(np.linalg.norm(goal_pos - puck_pos))
            dists_final.append(np.linalg.norm(goal_final_pos - puck_pos))

            k_actual = np.minimum(des_pos.shape[0] - 1, k)

            cur_pos = self.sim.data.qpos[:7].copy()
            cur_vel = self.sim.data.qvel[:7].copy()
            actual_pos[k_actual, :] = cur_pos

            # Use MuJoCo's internal PD Controller
            gripper_ctrl = -1  # Keep gripper closed
            self.sim.data.ctrl[:] = np.concatenate((des_pos[k_actual], des_vel[k_actual], [gripper_ctrl, gripper_ctrl]))

            # Apply gravity compensation
            joint_indices = np.arange(1, 8)
            self.sim.data.qfrc_applied[joint_indices] = self.sim.data.qfrc_bias[joint_indices]

            # Forward the simulation
            self.sim.step()

            # Render the scene
            if self.render and self.viewer is not None:
                self.viewer.render()

            k += 1

        if plot_trajectory:
            plt.figure()
            plt.plot(puck_poss)
            plt.pause(0.1)
            plt.figure()
            plt.plot(puck_vels)
            plt.pause(0.1)
            plt.figure()
            plt.plot(dists)
            plt.pause(0.1)

            # plt.figure()
            # plt.plot(np.vstack(torques))
            # plt.pause(0.1)
        min_dist = np.min(dists)
        return dists[-1]**2 + np.linalg.norm(puck_vel)
