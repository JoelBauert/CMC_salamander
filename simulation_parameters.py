"""Simulation parameters"""

import numpy as np
class SimulationParameters:
    """Simulation parameters"""

    def __init__(self, drive=0, coupled=True, **kwargs):
        super(SimulationParameters, self).__init__()
        # Default parameters
        self.n_body_joints = 8
        self.n_legs_joints = 4
        self.duration = 30
        self.nbr_lag = 2*np.pi/self.n_body_joints # phase lag on neighboring oscillator
        self.contra_lag = np.pi # phase lag between contralateral
        self.limb_lag = np.pi # phase lag involving limb
        self.amplitude_gradient = None
        self.phase_lag = None
        # Feel free to add more parameters (ex: MLR drive)
        self.rate = 20 # 1/s, amplitude converge rate
        self.r_sat = 0 # radian, saturation of R
        self.v_sat = 0 # Hz, saturation of f
        self.w_body = 10 # body coupling
        self.w_bodylimb = 30 # body to limb coupling
        self.w_limb = 10 # limb coupling
        self.c_v_body = [0.2, 0.3] # coefficients for body frequency
        self.c_v_limb = [0.2, 0.0] # coefficients for limb frequency
        self.c_r_body = [0.065, 0.196] # coefficients for body amplitude
        self.c_r_limb = [0.131, 0.131] # coefficients for limb amplitude
        self.d_body = [1.0, 5.0] # coefficients for body drive
        self.d_limb = [1.0, 3.0] # coefficients for limb drive
        self.drive = drive*np.ones(2 * self.n_body_joints + self.n_legs_joints) # this is for drive applied to all oscillators
        self.coupled = coupled

        self.phase_bias_downward = -2*np.pi/8
        self.phase_bias_upward = 2*np.pi/8
        self.phase_bias_contralateral = np.pi
        self.phase_bias_limb_to_body = np.pi
        self.phase_bias_limb_to_limb = np.pi

        # ...
        # Update object with provided keyword arguments
        # NOTE: This overrides the previous declarations
        self.__dict__.update(kwargs)

