from Experiment import Experiment
from simulator.default_settings import *

WIDTH = 500
HEIGHT = 500

# Initialize experiment
exp = Experiment(WIDTH, HEIGHT, centered=True, gui=True, use_mouse=USE_MOUSE)
# Environment settings
exp.env_settings()
# Loop
exp.main_loop()
