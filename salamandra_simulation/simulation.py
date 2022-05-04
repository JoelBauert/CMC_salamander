"""Simulation"""

import numpy as np

from farms_core import pylog
from farms_core.model.options import ArenaOptions
from farms_core.simulation.options import Simulator, SimulationOptions
from farms_sim.simulation import simulation_setup

from salamandra_simulation.options import SalamandraOptions
from salamandra_simulation.data import SalamandraData
from salamandra_simulation.controller import SalamandraController
from salamandra_simulation.callbacks import SwimmingCallback
from salamandra_simulation.camera import CameraCallback, save_video
from network import SalamandraNetwork


def simulation(
        sim_parameters,
        animat_config='salamandra_config_default.yaml',
        arena='water',
        simulator=Simulator.MUJOCO,
        verbose=False,
        **kwargs,
):
    """Main"""

    record_path = kwargs.pop('record_path', 'simulation')

    if not verbose:
        pylog.set_level('warning')

    # Simulation options
    pylog.info('Creating simulation')
    n_iterations = int(sim_parameters.duration/sim_parameters.timestep)
    simulation_options = SimulationOptions.with_clargs(
        timestep=sim_parameters.timestep,
        n_iterations=n_iterations,
        **kwargs,
    )

    # Arena options
    if arena == 'water':
        water_height = 0
        arena_options = ArenaOptions.load('arena_water_config.yaml')
    elif arena == 'amphibious':
        water_height = -0.1
        arena_options = ArenaOptions.load('arena_amphibious_config.yaml')
    else:
        water_height = -np.inf
        arena_options = ArenaOptions.load('arena_flat_config.yaml')
    if arena_options.water.sdf:
        arena_options.water.height = water_height

    # Animat options
    animat_options = SalamandraOptions.load(animat_config)
    animat_options.spawn.pose[:3] = sim_parameters.spawn_position
    animat_options.spawn.pose[3:] = sim_parameters.spawn_orientation
    animat_options.physics.water_height = water_height

    # Data
    animat_data = SalamandraData.from_options(
        animat_options=animat_options,
        simulation_options=simulation_options,
    )

    # Network
    network = SalamandraNetwork(
        sim_parameters=sim_parameters,
        n_iterations=n_iterations,
        state=animat_data.state,
    )

    # Controller
    animat_controller = SalamandraController(
        joints_names=animat_options.control.joints_names(),
        animat_data=animat_data,
        network=network,
    )

    # Other options
    options = {}

    # Callbacks
    if simulator == Simulator.MUJOCO:
        options['callbacks'] = [SwimmingCallback(animat_options)]
        if simulation_options.record:
            camera = CameraCallback(
                timestep=sim_parameters.timestep,
                n_iterations=n_iterations,
                fps=30,
            )
            options['callbacks'] += [camera]

    # Setup simulation
    sim = simulation_setup(
        animat_data=animat_data,
        arena_options=arena_options,
        animat_options=animat_options,
        simulation_options=simulation_options,
        animat_controller=animat_controller,
        simulator=simulator,
        **options,
    )

    # Run simulation
    pylog.info('Running simulation')
    sim.run()

    # Record
    if simulation_options.record:
        save_video(
            camera=camera,
            iteration=sim.iteration,
            record_path=record_path,
        )

    if not verbose:
        pylog.set_level('debug')

    return sim, animat_data

