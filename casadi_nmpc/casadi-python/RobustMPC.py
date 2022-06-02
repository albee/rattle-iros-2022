"""
RobustMPC

CasADi interface to a robust tube MPC.
"""

import casadi


class RobustMPC:
    def __init__(self, casadi_func):
        """Initialize with the CasADi serialized function.
        """
        self.casadi_func = casadi_func
        self.dynamics = 'DoubleIntegrator'

    def call_robust_mpc():
        pass

    def update_robust_mpc():
        """Update RobustMPC based on latest parameter estimates.
        """
        pass
