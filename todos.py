"""
• Ask Pascal why he is not using the MuJoCo internal controller -> Use internal controller
• Find out the p/d gains for controller -> Ask Li
• Make three different difficulties:
    • Directly apply force to puck
    • Only use 3 joints
    • Use all joints
• Find out a good initial position -> [0, 0.202, 0, -2.86, 0, 1.98, 0.771, 0, 0]
• Write a .xml parser for initial puck position, puck size -> Done
• dt? substeps? -> Use 500Hz and 5 substeps = 100 Hz control frequency
• Add a collision check function -> Done
• Add the multiprocessing part
• Add robot_final_qpos as a parameter to rollout()
"""

"""20.10.2020
• PushPuckNoRobot: 2D input action, rollout apply force/impulse
• PushPuck2:
    • rollout input weights + goal pos
    • collision check
    • max episode length class attribute (200...)
• PushPuck_v3: fully parametrized to actuate all joints
    • Puck goal position as input for init
• Parallel rollouts"
"""
