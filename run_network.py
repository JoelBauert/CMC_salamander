"""Run network without MuJoCo"""

from cmath import pi
import time
import numpy as np
import matplotlib.pyplot as plt
from farms_core import pylog
from salamandra_simulation.data import SalamandraState
from salamandra_simulation.parse_args import save_plots
from salamandra_simulation.save_figures import save_figures
from simulation_parameters import SimulationParameters
from network import SalamandraNetwork


def run_network(duration, update=False, drive=0, coupled=True):
    """Run network without MuJoCo and plot results
    Parameters
    ----------
    duration: <float>
        Duration in [s] for which the network should be run
    update: <bool>
        description
    drive: <float/array>
        Central drive to the oscillators
    """
    # Simulation setup
    timestep = 1e-2
    times = np.arange(0, duration, timestep)
    n_iterations = len(times)
    sim_parameters = SimulationParameters(
        drive=drive,
        coupled=coupled,
        amplitude_gradient=None,
        phase_lag=None,
        turn=None,
    )
    state = SalamandraState.salamandra_robotica_2(n_iterations)
    network = SalamandraNetwork(sim_parameters, n_iterations, state)
    osc_left = np.arange(8)
    osc_right = np.arange(8, 16)
    osc_legs = np.arange(16, 20)

    # Logs
    phases_log = np.zeros([
        n_iterations,
        len(network.state.phases(iteration=0))
    ])
    phases_log[0, :] = network.state.phases(iteration=0)
    amplitudes_log = np.zeros([
        n_iterations,
        len(network.state.amplitudes(iteration=0))
    ])
    amplitudes_log[0, :] = network.state.amplitudes(iteration=0)
    freqs_log = np.zeros([
        n_iterations,
        len(network.robot_parameters.freqs)
    ])
    freqs_log[0, :] = network.robot_parameters.freqs
    drive_log = np.zeros([
        n_iterations, 
        len(network.robot_parameters.var_drive)
    ])
    drive_log[0, :] = network.robot_parameters.var_drive
    outputs_log = np.zeros([
        n_iterations,
        len(network.get_motor_position_output(iteration=0))
    ])
    outputs_log[0, :] = network.get_motor_position_output(iteration=0)

    # Run network ODE and log data
    tic = time.time()
    for i, time0 in enumerate(times[1:]):
        if update:
            network.robot_parameters.update(
                SimulationParameters(
                    drive = drive + i*0.00125,
                    coupled = coupled
                    # amplitude_gradient=None,
                    # phase_lag=None
                )
            )
        network.step(i, time0, timestep)
        phases_log[i+1, :] = network.state.phases(iteration=i+1)
        amplitudes_log[i+1, :] = network.state.amplitudes(iteration=i+1)
        outputs_log[i+1, :] = network.get_motor_position_output(iteration=i+1)
        freqs_log[i+1, :] = network.robot_parameters.freqs
        drive_log[i+1, :] = network.robot_parameters.var_drive
    # # Alternative option
    # phases_log[:, :] = network.state.phases()
    # amplitudes_log[:, :] = network.state.amplitudes()
    # outputs_log[:, :] = network.get_motor_position_output()
    toc = time.time()

    # Network performance
    pylog.info('Time to run simulation for {} steps: {} [s]'.format(
        n_iterations,
        toc - tic
    ))

    # Implement plots of network results
    if coupled:
        fig3, ax = plt.subplots(4, sharex=True)
        plt.xlabel('Time(ms)')
        shift = 0 
        for i in range(8):
            ax[0].plot(times, (outputs_log[:, i] - shift))
            shift += 2
        ax[0].set_ylabel('Body Output')

        shift = 0 
        for i in range(2):
            ax[1].plot(times, (outputs_log[:, i+16] - shift))
            shift += 1
        ax[1].set_ylabel('Limb Output')

        ax[2].plot(times, freqs_log[:, :])
        ax[2].set_ylabel('Frquency [Hz]')

        ax[3].plot(times, drive_log[:, :])
        ax[3].set_ylabel('drive d')
    else:
        fig4left, ax = plt.subplots(2, sharex=True)
        plt.xlabel('drive')
        ax[0].plot(drive_log[:, 1], freqs_log[:, 1])
        ax[0].plot(drive_log[:, 1], freqs_log[:, 16])
        ax[0].set_ylabel('v [Hz]')

        ax[1].plot(drive_log[:, 1], amplitudes_log[:, 1])
        ax[1].plot(drive_log[:, 1], amplitudes_log[:, 16])
        ax[1].set_ylabel('R')
        
        fig4right, ax = plt.subplots(4, sharex=True)
        plt.xlabel('Time(ms)')
        shift = 2
        ax[0].plot(times, (outputs_log[:, 1]))
        ax[0].plot(times, (outputs_log[:, 16] - shift))
        ax[0].set_ylabel('Outputs')

        ax[1].plot(times, freqs_log[:, 1])
        ax[1].plot(times, freqs_log[:, 16])
        ax[1].set_ylabel('Frquency [Hz]')

        ax[2].plot(times, amplitudes_log[:, 1])
        ax[2].plot(times, amplitudes_log[:, 16])
        ax[2].set_ylabel('r')

        ax[3].plot(times, drive_log[:, :])
        ax[3].set_ylabel('drive d')


def main(plot):
    """Main"""
    drive_ = 0.5 #initial drive (or constant drive matrix if update=False)
    run_network(duration=40, update=True, drive=drive_, coupled=False)

    # Show plots
    if plot:
        plt.show()
    else:
        save_figures()


if __name__ == '__main__':
    main(plot=not save_plots())

