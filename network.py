"""Oscillator network ODE"""

import numpy as np
from scipy.integrate import ode
from robot_parameters import RobotParameters


def network_ode(_time, state, robot_parameters):
    """Network_ODE

    Parameters
    ----------
    _time: <float>
        Time
    state: <np.array>
        ODE states at time _time
    robot_parameters: <RobotParameters>
        Instance of RobotParameters

    Returns
    -------
    :<np.array>
        Returns derivative of state (phases and amplitudes)

    """
    n_oscillators = robot_parameters.n_oscillators
    phases = state[:n_oscillators]
    amplitudes = state[n_oscillators:2*n_oscillators]
    # Implement ODE:
    phase_diff = phases - np.repeat(np.expand_dims(phases, axis=1), n_oscillators, axis=1) # matrix of phase difference
    sin_mat = np.sin(phase_diff - robot_parameters.phase_bias) # get the matrix for sin values in the ode
    d_phase = 2 * np.pi * robot_parameters.freqs + (sin_mat * robot_parameters.coupling_weights) @ amplitudes
    d_amp = robot_parameters.rates * (robot_parameters.nominal_amplitudes - amplitudes)
    
    return np.concatenate([d_phase, d_amp])


def motor_output(phases, amplitudes, iteration):
    """Motor output

    Parameters
    ----------
    phases: <np.array>
        Phases of the oscillator
    amplitudes: <np.array>
        Amplitudes of the oscillator

    Returns
    -------
    : <np.array>
        Motor outputs for joint in the system.

    """
    # Implement equation here
    n_body=16
    n_legs=4 
    # the above two parameters are hard-coded here
    left_body = amplitudes[:8] * (1 + np.cos(phases[:8])) - \
                amplitudes[8:16] * (1 + np.cos(phases[8:16]))
    
    limb = amplitudes[-4:] * (1 + np.cos(phases[-4:]))
    
    return np.concatenate([left_body, -left_body, limb])


class SalamandraNetwork:
    """Salamandra oscillator network"""

    def __init__(self, sim_parameters, n_iterations, state):
        super().__init__()
        self.n_iterations = n_iterations
        # States
        self.state = state
        # Parameters
        self.robot_parameters = RobotParameters(sim_parameters)
        # Set initial state
        # Replace your oscillator phases here
        self.state.set_phases(
            iteration=0,
            value=1e-4*np.random.ranf(self.robot_parameters.n_oscillators),
        )
        # Set solver
        self.solver = ode(f=network_ode)
        self.solver.set_integrator('dopri5')
        self.solver.set_initial_value(y=self.state.array[0], t=0.0)

    def step(self, iteration, time, timestep):
        """Step"""
        if iteration + 1 >= self.n_iterations:
            return
        self.solver.set_f_params(self.robot_parameters)
        self.state.array[iteration+1, :] = self.solver.integrate(time+timestep)

    def outputs(self, iteration=None):
        """Oscillator outputs"""
        # Implement equation here
        return np.zeros(12)

    def get_motor_position_output(self, iteration=None):
        """Get motor position"""
        return motor_output(
            self.state.phases(iteration=iteration),
            self.state.amplitudes(iteration=iteration),
            iteration=iteration,
        )

