"""
# test_rattle_coordinator.py

Check weighting assigments.
"""
# %%
# Required to perform an absolute import of a different package
import sys
import os
sys.path.insert(0, os.path.abspath('..'))

from src.rattle_coordinator import RattleCoordinator
import numpy as np

rc = RattleCoordinator()

# normal
sigma_floor = np.array([0.01, 0.01, 0.01, 0.01])
sigma = np.array([1.0, 1.0, 1.0, 1.0])
weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])

rc.get_weights_covar(sigma_floor, sigma, weight_vec0)

# approaching
sigma_floor = np.array([0.01, 0.01, 0.01, 0.01])
sigma = np.array([0.05, 0.05, 0.05, 0.05])
weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])

rc.get_weights_covar(sigma_floor, sigma, weight_vec0)

# near floor
sigma_floor = np.array([0.01, 0.01, 0.01, 0.01])
sigma = np.array([0.012, 0.012, 0.012, 0.012])
weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])

rc.get_weights_covar(sigma_floor, sigma, weight_vec0)

# %%
