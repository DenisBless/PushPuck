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

"""5.11.2020
• Goal pos parsing (Needs to be in code since the xml is loaded at the beginning)
• Multiprocessing -> Done
• 1 Class with call function which takes weight batch and returns Rewards from all samples -> Done
"""

"""Questions
-Where should we specify the puck_pos/size? Is it different for trajectories or put it in __init()__? 
-What if target pos is None?
-How is the size of the weight vector determined? DoF * 5?
"""

"""Notes
Added helper for getting the target, puck and end effector position.
"""
