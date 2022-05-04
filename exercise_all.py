"""Python controller"""

from farms_core import pylog
from exercise_example import exercise_example


def exercise_all(arguments):
    """Run all exercises"""

    verbose = 'not_verbose' not in arguments

    if not verbose:
        pylog.set_level('warning')

    # Timestep
    timestep = 1e-2

    # Exercise example to show how to run a grid search
    if 'exercise_example' in arguments:
        exercise_example(timestep)

    if not verbose:
        pylog.set_level('debug')


if __name__ == '__main__':
    exercise_all(arguments=['example'])

