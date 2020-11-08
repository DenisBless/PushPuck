from multiprocessing import Process, Array, Lock
from push_puck.push_puck_robot import PushPuck3DoF, PushPuck7DoF
from push_puck.push_puck_no_robot import PushPuckNoRobot
import numpy as np
import math


class PushPuckMultiProcessing:
    def __init__(self, push_puck, n_workers, n_substeps=1, render=False) -> None:
        self.push_puck = push_puck
        self.n_workers = n_workers
        self.n_substeps = n_substeps
        self.render = render
        self.lock = Lock()

    def work(self, weights, goal_pos, pid, reward_arr, lock, n_samples_per_worker):
        n_runs = weights.shape[0]
        with lock:
            pp = self.push_puck(n_substeps=self.n_substeps, render=self.render)  # Create instance of PushPuck obj
        for run in range(n_runs):
            reward = pp.rollout(weights[run], goal_pos[run])
            reward_arr[n_samples_per_worker * pid + run] = reward

    def __call__(self, weights, goal_pos):
        n_samples = weights.shape[0]

        n_samples_per_worker = math.ceil(n_samples / self.n_workers)
        rewards = Array('d', [0] * n_samples)

        processes = [Process(target=self.work,
                             args=(
                                 weights[n_samples_per_worker * pid: n_samples_per_worker * pid + n_samples_per_worker],
                                 goal_pos[n_samples_per_worker * pid: n_samples_per_worker * pid + n_samples_per_worker],
                                 pid, rewards, self.lock, n_samples_per_worker))
                     for pid in range(self.n_workers)]

        for p in processes:
            p.start()

        for p in processes:
            p.join()

        return np.array(rewards)


if __name__ == '__main__':
    pp_mp = PushPuckMultiProcessing(push_puck=PushPuckNoRobot, n_workers=6, render=False)
    rewards = pp_mp(weights=np.random.randn(32, 2), goal_pos=np.zeros([32, 1]))
    print(rewards)
