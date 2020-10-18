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
"""