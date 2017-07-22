from Experiment import Experiment
from simulator.settings import *

WIDTH = 800
HEIGHT = 600

# Initialize experiment
exp = Experiment(WIDTH, HEIGHT, centered=True, gui=True)
# Environment settings
exp.env_settings()
# Loop
exp.main_loop()
