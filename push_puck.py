import mujoco_py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mujoco_py import MjSim, MjViewer, load_model_from_path, MjSimState
from utils.helpers import set_puck
from mp_lib.dmps import DMP
from mp_lib.phase import ExpDecayPhaseGenerator
from mp_lib.basis import DMPBasisGenerator
from abc import abstractmethod, ABC


class PushPuckBase(ABC):
    def __init__(self,
                 nsubsteps: int = 1,
                 render: bool = True,
                 num_dof: int = 7):
        self.render = render

        set_puck(raw_xml_path=self.raw_xml_path, xml_path=self.xml_path, puck_size=None, puck_pos=None)
        model = load_model_from_path(self.xml_path)

        self.sim = MjSim(model=model, nsubsteps=nsubsteps)
        self.viewer = MjViewer(self.sim) if render else None

        self.robot_init_qpos = np.array([0, 0.202, 0, -2.86, 0, 1.98, 0.771, 0, 0])
        self.robot_final_qpos = np.array([0, 1.2, 0, -2.86, 0, 4.1, 0.771, 0, 0])

        self.joint_indices = [x for x in range(1, num_dof + 1)]

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

    @abstractmethod
    def rollout(self, weights):
        raise NotImplementedError


class PushPuck2(PushPuckBase):
    def __init__(self,
                 nsubsteps: int = 1,
                 render: bool = True,
                 num_dof: int = 7):
        super().__init__(nsubsteps=nsubsteps, render=render, num_dof=num_dof)

    @property
    def raw_xml_path(self):
        return str(Path(__file__).resolve().parents[0]) + '/assets/xml_model/env_model_raw.xml'

    def rollout(self, weights):
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
                  dt=dt
                  )

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

            cur_pos = self.sim.data.qpos[0:7].copy()
            cur_vel = self.sim.data.qvel[0:7].copy()
            actual_pos[k_actual, :] = cur_pos

            # Use MuJoCo's internal PD Controller
            gripper_ctrl = -1  # Keep gripper closed
            self.sim.data.ctrl[:] = np.concatenate((des_pos[k_actual], des_vel[k_actual], [gripper_ctrl, gripper_ctrl]))

            # Apply gravity compensation
            self.sim.data.qfrc_applied[self.joint_indices] = self.sim.data.qfrc_bias[self.joint_indices]

            # Forward the simulation
            self.sim.step()

            # Render the scene
            if self.render and self.viewer is not None:
                self.viewer.render()

            k += 1

            """
            Old ctrl loop
            # Compute the controls
            cur_pos = self.sim.data.qpos[0:7].copy()
            cur_vel = self.sim.data.qvel[0:7].copy()
            k_actual = np.minimum(des_pos.shape[0] - 1, k)
            trq = self.p_gains * (des_pos[k_actual, :] - cur_pos) + self.d_gains * (des_vel[k_actual, :] - cur_vel)
            torques.append(trq)

            actual_pos[k_actual, :] = cur_pos

            # Advance the simulation
            self.sim.data.qfrc_applied[0:7] = trq
            
            
            
            try:
                self.sim.step()
            except mujoco_py.builder.MujocoException as e:
                print("Error in simulation: " + str(e))
                error = True
                # Copy the current torque as if it would have been applied until the end of the trajectory
                for i in range(k + 1, des_pos.shape[0] + 200):
                    torques.append(trq)
                break
            

            k += 1

            # # Check for a collision - in which case we end the simulation
            # if BallInACupCostFunction._check_collision(self.sim, ball_collision_id, collision_ids):
            #     # Copy the current torque as if it would have been applied until the end of the trajectory
            #     for i in range(k + 1, des_pos.shape[0]):
            #         torques.append(trq)
            #     break
            

            if viewer is not None:
                viewer.render()

        if viewer is not None:
            viewer.close()
        """

        if plot_trajectory:
            plt.figure()
            plt.plot(actual_pos)
            plt.pause(0.1)

            # plt.figure()
            # plt.plot(np.vstack(torques))
            # plt.pause(0.1)
        min_dist = np.min(dists)
        return 0


if __name__ == '__main__':
    pp = PushPuck2(nsubsteps=5, render=True)
    # Only make joint 2,4 and 6 controllable!

    w = 50 * np.random.randn(15)
    pp.rollout(w)
    # for i in range(10000):
    #     pp.viewer.render()
