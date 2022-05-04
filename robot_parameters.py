"""Robot parameters"""

import numpy as np
from farms_core import pylog


class RobotParameters(dict):
    """Robot parameters"""

    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__

    def __init__(self, parameters):
        super(RobotParameters, self).__init__()

        # Initialise parameters
        self.n_body_joints = parameters.n_body_joints
        self.n_legs_joints = parameters.n_legs_joints
        self.n_joints = self.n_body_joints + self.n_legs_joints
        self.n_oscillators_body = 2*self.n_body_joints
        self.n_oscillators_legs = self.n_legs_joints
        self.n_oscillators = self.n_oscillators_body + self.n_oscillators_legs
        self.freqs = np.zeros(self.n_oscillators)
        self.coupling_weights = np.zeros([
            self.n_oscillators,
            self.n_oscillators,
        ])
        self.phase_bias = np.zeros([self.n_oscillators, self.n_oscillators])
        self.rates = np.zeros(self.n_oscillators)
        self.nominal_amplitudes = np.zeros(self.n_oscillators)
        self.var_drive = np.zeros(2 * self.n_body_joints + self.n_legs_joints)
        self.update(parameters)

    def update(self, parameters):
        """Update network from parameters"""
        self.set_frequencies(parameters)  # f_i
        self.set_coupling_weights(parameters)  # w_ij
        self.set_phase_bias(parameters)  # theta_i
        self.set_amplitudes_rate(parameters)  # a_i
        self.set_nominal_amplitudes(parameters)  # R_i
        self.set_drive(parameters)

    def set_frequencies(self, parameters):
        """Set frequencies"""
        #pylog.warning('Coupling weights must be set')
        for i, d in enumerate(self.var_drive):

            if parameters.coupled:
                if (d > parameters.d_body[0]) & (d < parameters.d_body[1]):
                    if (d > parameters.d_limb[0]) & (d < parameters.d_limb[1]):
                        self.freqs[i] = parameters.c_v_limb[0] * d + parameters.c_v_limb[1]
                    else:
                        if i < self.n_oscillators_body:
                            self.freqs[i] = parameters.c_v_body[0] * d + parameters.c_v_body[1]
                        else:
                            self.freqs[i] = parameters.v_sat
                else:
                    self.freqs[i] = parameters.v_sat

            else:
                if i < self.n_oscillators_body:
                    if (d > parameters.d_body[0]) & (d < parameters.d_body[1]):
                        self.freqs[i] = parameters.c_v_body[0] * d + parameters.c_v_body[1]
                    else:
                        self.freqs[i] = parameters.v_sat
                else:
                    if (d > parameters.d_limb[0]) & (d < parameters.d_limb[1]):
                        self.freqs[i] = parameters.c_v_limb[0] * d + parameters.c_v_limb[1]
                    else:
                        self.freqs[i] = parameters.v_sat
            
        #print('updated frequency:\n {}'.format(self.freqs))
        

    def set_coupling_weights(self, parameters):
        """Set coupling weights"""
        #pylog.warning('Coupling weights must be set')
        body_left = np.eye(self.n_body_joints, k=1) + np.eye(self.n_body_joints, k=-1) # block matrix for one side connection
        w_body = parameters.w_body * (np.block([[body_left, np.zeros((self.n_body_joints, self.n_body_joints))], \
                                                [np.zeros((self.n_body_joints, self.n_body_joints)), body_left]]) + \
                                      np.eye(self.n_oscillators_body, k=self.n_body_joints) + \
                                      np.eye(self.n_oscillators_body, k=-self.n_body_joints))
        
        w_limb = parameters.w_limb * np.array([[0, 1, 1, 0], \
                                               [1, 0, 0, 1], \
                                               [1, 0, 0, 1], \
                                               [0, 1, 1, 0]]) # this part is hard coded for 4 limbs
        a = np.zeros((4, 1)) # assist for creating the next matrix
        w_bodylimb = parameters.w_bodylimb * np.block([[a+1, a, a, a], \
                                                       [a, a, a+1, a], \
                                                       [a, a+1, a, a], \
                                                       [a, a, a, a+1]]) # this part is hard coded for 4 limbs
        
        self.coupling_weights = np.block([[w_body, w_bodylimb], \
                                          [np.zeros((self.n_oscillators_legs, self.n_oscillators_body)), w_limb]])
        

    def set_phase_bias(self, parameters):
         #Phase bias withing body CPG
         for i in range(self.n_body_joints):
             self.phase_bias[i][i+self.n_body_joints] = parameters.phase_bias_contralateral
             self.phase_bias[i+self.n_body_joints][i] = parameters.phase_bias_contralateral
             if i<range(self.n_body_joints)[-1]:
                 self.phase_bias[i][i+1] = parameters.phase_bias_downward
                 self.phase_bias[i+self.n_body_joints][i+self.n_body_joints+1] = parameters.phase_bias_downward
                 self.phase_bias[i+1][i] = parameters.phase_bias_upward
                 self.phase_bias[i+1+self.n_body_joints][i+self.n_body_joints] = parameters.phase_bias_upward
         #Now we define phase bias within limbs
         self.phase_bias[2*self.n_body_joints][2*self.n_body_joints+1] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+1][2*self.n_body_joints] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints][2*self.n_body_joints+2] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+2][2*self.n_body_joints] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+3][2*self.n_body_joints+1] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+1][2*self.n_body_joints+3] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+3][2*self.n_body_joints+2] = parameters.phase_bias_limb_to_limb  
         self.phase_bias[2*self.n_body_joints+2][2*self.n_body_joints+3] = parameters.phase_bias_limb_to_limb  
         #And then phase bias between limb and body
         for j in range(4):
             self.phase_bias[2*self.n_body_joints][j] = parameters.phase_bias_limb_to_body
             self.phase_bias[2*self.n_body_joints+1][self.n_body_joints+j] = parameters.phase_bias_limb_to_body
             self.phase_bias[2*self.n_body_joints+2][round(self.n_body_joints/2)+j] = parameters.phase_bias_limb_to_body
             self.phase_bias[2*self.n_body_joints+3][round(3/2*(self.n_body_joints))+j] = parameters.phase_bias_limb_to_body

        
    def set_amplitudes_rate(self, parameters):
        """Set amplitude rates"""
        #pylog.warning('Convergence rates must be set')
        self.rates = parameters.rate

    def set_nominal_amplitudes(self, parameters):
        """Set nominal amplitudes"""
        #pylog.warning('Nominal amplitudes must be set')
        for i, d in enumerate(self.var_drive):

            if i < self.n_oscillators_body:
                if (d > parameters.d_body[0]) & (d < parameters.d_body[1]):
                    self.nominal_amplitudes[i] = parameters.c_r_body[0] * d + parameters.c_r_body[1]
                else:
                    self.nominal_amplitudes[i] = parameters.r_sat
            else:
                if (d > parameters.d_limb[0]) & (d < parameters.d_limb[1]):
                    self.nominal_amplitudes[i] = parameters.c_r_limb[0] * d + parameters.c_r_limb[1]
                else:
                    self.nominal_amplitudes[i] = parameters.r_sat
        #print('updated amplitudes:\n {}'.format(self.nominal_amplitudes))

    def set_drive(self, parameters):
        """set the drive"""
        self.var_drive = parameters.drive
        #print('updated drive:\n {}'.format(self.var_drive))

