"""Animat data"""

from typing import Dict, Any

import numpy as np
from nptyping import NDArray

from farms_core import pylog
from farms_core.io.hdf5 import hdf5_to_dict
from farms_core.model.data import AnimatData
from farms_core.model.options import AnimatOptions
from farms_core.simulation.options import SimulationOptions
from farms_core.sensors.data import SensorsData

NPDTYPE = np.float64
NPITYPE = np.uintc


def to_array(array, iteration=None):
    """To array or None"""
    if array is not None:
        array = np.array(array)
        if iteration is not None:
            array = array[:iteration]
    return array


class SalamandraState:
    """Salamandra state"""

    def __init__(self, array):
        super().__init__()
        self.array = array

    @classmethod
    def salamandra_robotica_2(cls, n_iterations):
        """State of Salamandra robotica 2"""
        return cls(array=np.zeros([n_iterations, 2*20]))

    def phases(self, iteration=None):
        """Oscillator phases"""
        return (
            self.array[iteration, :20]
            if iteration is not None
            else self.array[:, :20]
        )

    def set_phases(self, iteration, value):
        """Set phases"""
        self.array[iteration, :20] = value

    def set_phases_left(self, iteration, value):
        """Set body phases on left side"""
        self.array[iteration, :8] = value

    def set_phases_right(self, iteration, value):
        """Set body phases on right side"""
        self.array[iteration, 8:16] = value

    def set_phases_legs(self, iteration, value):
        """Set leg phases"""
        self.array[iteration, 16:20] = value

    def amplitudes(self, iteration=None):
        """Oscillator amplitudes"""
        return (
            self.array[iteration, 20:]
            if iteration is not None
            else self.array[:, 20:]
        )

    def set_amplitudes(self, iteration, value):
        """Set amplitudes"""
        self.array[iteration, 20:] = value


class SalamandraData(AnimatData):
    """Salamandra data"""

    def __init__(
            self,
            state: SalamandraState,
            **kwargs,
    ):
        super().__init__(**kwargs)
        self.state: SalamandraState = state

    @classmethod
    def from_options(
            cls,
            animat_options: AnimatOptions,
            simulation_options: SimulationOptions,
    ):
        """From options"""

        # Sensors
        sensors = SensorsData.from_options(
            animat_options=animat_options,
            simulation_options=simulation_options,
        )

        # State
        state = SalamandraState.salamandra_robotica_2(
            n_iterations=simulation_options.n_iterations,
        )

        return cls(
            timestep=simulation_options.timestep,
            sensors=sensors,
            state=state,
        )

    @classmethod
    def from_file(cls, filename: str):
        """From file"""
        pylog.info('Loading data from %s', filename)
        data = hdf5_to_dict(filename=filename)
        pylog.info('loaded data from %s', filename)
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, dictionary: Dict):
        """Load data from dictionary"""
        return cls(
            timestep=dictionary['timestep'],
            sensors=SensorsData.from_dict(dictionary['sensors']),
            state=SalamandraState(array=dictionary['state']),
        )

    def to_dict(self, iteration: int = None) -> Dict:
        """Convert data to dictionary"""
        data_dict = super().to_dict(iteration=iteration)
        data_dict.update({'state': to_array(self.state.array)})
        return data_dict

    def plot(self, times: NDArray[(Any,), float]) -> Dict:
        """Plot"""
        plots = {}
        plots.update(self.plot_sensors(times))
        return plots

