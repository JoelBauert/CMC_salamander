""" Lab 6 """

from multiprocessing import Pool
from farms_core import pylog
from salamandra_simulation.parse_args import save_plots
from run_network import main as run_network
from exercise_all import exercise_all
from plot_results import main as plot_results


def main(run_simulations=True, parallel=False, verbose=True):
    """Main function that runs all the exercises."""
    save = save_plots()
    pylog.info('Running network')
    run_network(plot=not save)
    pylog.info('Running simulation exercises')
    arguments = (
        ['exercise_example']
        if run_simulations
        else []
    )
    if not verbose:
        arguments += ['not_verbose']
    if parallel:
        with Pool() as pool:  # Pool(processes=4)
            pool.map(exercise_all, [[arg] for arg in arguments])
    else:
        exercise_all(arguments=arguments)
    pylog.info('Plotting simulation results')
    plot_results(plot=not save)


if __name__ == '__main__':
    main()

