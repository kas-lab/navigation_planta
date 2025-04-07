from pathlib import Path
from datetime import datetime
import time
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

from map_generator import MapGenerator
from prism_model_generator import PrismModelgenerator

class Runner:
    def __init__(self, map_generator_class=MapGenerator, result_folder="results"):
        self.map_generator = map_generator_class()
        self.result_folder = Path(result_folder)
        self.execution_times = []
        self.experiment_header = ''

    def load_json_map(self, json_path='map_camara_2020_paper/map-p2cp3.json'):
        self.map_generator.load_json(json_path)

    def execute_case(self, map_folder: Path, init: int, goal: int) -> float:
        raise NotImplementedError("Subclasses must implement this method.")
    
    def run_case_wrapper(self, folder_name, run_id, init, goal):
        raise NotImplementedError("Subclasses must implement this method.")

    def run_all_cases(self,  n_runs) -> list[Tuple[str, str]]:
        print('You need to overwrite the run_all_cases method!!!!')
        return [('', '')]

    def run_experiment(self, n_runs=1):
        """Runs the full set of experiments and saves results."""
        folder_name = Path(self.result_folder)
        folder_name.mkdir(parents=True, exist_ok=True)

        self.execution_times = self.run_all_cases(folder_name, n_runs)

        planning_time_csv = folder_name / 'planning_times.csv'
        planning_time_array = np.array(self.execution_times, dtype=[(self.experiment_header, "U15"), ("time", "f8")])
        np.savetxt(planning_time_csv, planning_time_array, delimiter=",", header=f"{self.experiment_header},time", comments="", fmt="%s,%.18e")

        mean = np.mean(planning_time_array["time"])
        std_dev = np.std(planning_time_array["time"])
        with open(folder_name / 'results', 'w') as result_file:
            result_file.write('mean, standard deviation\n')
            result_file.write(f'{mean}, {std_dev}')
        print(f'Mean {mean} and Std dev: {std_dev}')

    def _run_subprocess(self, command: list[str]) -> subprocess.CompletedProcess:
        try:
            result = subprocess.run(
                command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return result
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Command failed: {e.cmd}")
            print(e.stderr)
            raise

class GridMapRunner(Runner):
    def __init__(
            self, map_generator_class=MapGenerator, result_folder="results/grid_map",
            min_nodes = 10,
            max_nodes = 1000,
            nodes_interval = 10,
            nodes_skip = 0.1,
            unconnected_amount = 0.15, 
            unsafe_amount = 0.25,
            dark_amount = 0.25):
        super().__init__(map_generator_class, result_folder)
        self.min_nodes = min_nodes
        self.max_nodes = max_nodes
        self.nodes_interval = nodes_interval
        self.nodes_skip = nodes_skip
        self.unconnected_amount = unconnected_amount
        self.unsafe_amount = unsafe_amount
        self.dark_amount = dark_amount
        self.experiment_header = 'nodes'
    
    def run_all_cases(self, folder_name, n_runs):
        
        execution_times = []

        for n_nodes in range(
            self.min_nodes,
            self.max_nodes +
            self.nodes_interval,
            self.nodes_interval):
            for n in range(n_runs):
                n_nodes_resulting, elapsed_time = self.execute_case(
                    folder_name, n, n_nodes)
                print(f"Nodes: {n_nodes_resulting} Execution Time: {elapsed_time:.6f} seconds")
                execution_times.append((n_nodes_resulting, elapsed_time))
        return execution_times
    
    def execute_case(self, 
        folder_name,
        run_n,
        n_nodes):

        n_nodes_resulting = n_nodes - int(n_nodes * self.nodes_skip)
        map_generator = MapGenerator(
            n_nodes,
            self.nodes_skip,
            self.unconnected_amount,
            self.unsafe_amount,
            self.dark_amount)
        map_generator.generate_connected_grid_map()

        map_folder_name = f'{n_nodes}_{self.nodes_skip}_{self.unconnected_amount}_{self.unsafe_amount}_{self.dark_amount}_{run_n}'
        map_folder = folder_name / map_folder_name
        map_folder.mkdir(parents=True, exist_ok=True)

        problem_filename = map_folder / 'problem.pddl'
        map_generator.generate_domain_problem_files(
            save_problem=True, problem_filename=problem_filename)

        map_filename = 'map.png'
        map_full_path = map_folder / map_filename
        map_generator.plot_graph(
            show_plot=False,
            save_file=True,
            filename=map_full_path)

        domain_output = map_folder / 'domain_created.pddl'
        problem_input = map_folder / 'problem.pddl'
        problem_output = map_folder / 'problem_created.pddl'

        owl_to_pddl_command = [
            "OWLToPDDL.sh",
            "--owl=owl/navigation.owl",
            "--tBox",
            "--inDomain=pddl/domain_sas.pddl",
            f"--outDomain={domain_output}",
            "--aBox",
            f"--inProblem={problem_input}",
            f"--outProblem={problem_output}",
            "--replace-output",
            "--add-num-comparisons"
        ]

        try:
            self._run_subprocess(owl_to_pddl_command)
        except subprocess.CalledProcessError as e:
            return self.execute_case(
                map_folder,
                folder_name,
                run_n,
                n_nodes)
        
        command = [
            "fast-downward.py",
            domain_output,
            problem_output,
            "--search", "astar(blind())"
        ]

        # Start high-precision timer
        start_time = time.perf_counter()
        
        self._run_subprocess(command)
        
        # Compute and print execution time
        return (n_nodes_resulting ,time.perf_counter() - start_time)
    
    def generate_save_plot(self, show=False):
        planning_time_array = np.array(
            self.execution_times, dtype=[
                ("nodes", "i4"), ("time", "f8")])
        unique_nodes, indices = np.unique(
        planning_time_array["nodes"], return_inverse=True)
        mean_times = np.bincount(
            indices, weights=planning_time_array["time"]) / np.bincount(indices)

        # Compute standard deviation for each group
        sum_squared_diffs = np.bincount(
            indices,
            weights=(
                planning_time_array["time"] -
                mean_times[indices])**2)
        std_dev_times = np.sqrt(sum_squared_diffs /
                                np.bincount(indices))  # Standard deviation formula

        # Upper and lower bounds
        upper_bound = mean_times + std_dev_times
        lower_bound = mean_times - std_dev_times

        # Create the plot
        fig, ax = plt.subplots(figsize=(8, 5))

        # Plot mean execution times
        ax.plot(
            unique_nodes,
            mean_times,
            marker='o',
            linestyle='-',
            color='b',
            label="Avg Execution Time")
        
        # Plot upper and lower bounds
        ax.plot(
            unique_nodes,
            upper_bound,
            linestyle='--',
            color='gray',
            label="Mean + Std Dev")
        
        ax.plot(
            unique_nodes,
            lower_bound,
            linestyle='--',
            color='gray',
            label="Mean - Std Dev")

        # Optionally: fill the area between upper and lower bounds
        ax.fill_between(
            unique_nodes,
            lower_bound,
            upper_bound,
            color='gray',
            alpha=0.2)
        
        # Labels and title
        ax.set_xlabel("Number of Nodes")
        ax.set_ylabel("Mean Elapsed Time (seconds)")
        ax.set_title("Average Planning Execution Time with Std Dev")
        ax.legend()
        ax.grid(True)

        # Save the plot
        planning_time_png = self.result_folder / 'planning_time_avg_std.png'
        plt.savefig(planning_time_png, format='png', dpi=300, bbox_inches='tight')

        # Show only this plot
        if show is True:
            plt.show(block=True)
            plt.close(fig)  # Ensure this figure is closed after display

class CamaraMapRunner(Runner):
    def __init__(self, map_generator_class=MapGenerator, result_folder="results"):
        super().__init__(map_generator_class, result_folder)
        self.unreachable_nodes = [20, 41]
        self.experiment_header = 'nodes'

    def run_case_wrapper(self, folder_name, run_id, init, goal):
        """Runs the PRISM command for a specific experiment instance."""
        
        map_folder_name = f'wp{init}_wp{goal}_{run_id}'
        map_folder = folder_name / map_folder_name
        map_folder.mkdir(parents=True, exist_ok=True)

        elapsed_time = self.execute_case(map_folder, init, goal)
        print(f"Execution Time: {elapsed_time:.6f} seconds")
        return (f'wp{init}_wp{goal}', elapsed_time)

    def run_all_cases(self, folder_name, n_runs):
        execution_times = []
        for init in range(1, 59):
            for goal in range(1, 59):
                if init != goal and init not in self.unreachable_nodes and goal not in self.unreachable_nodes:
                    for n in range(n_runs):
                        init_goal, elapsed_time = self.run_case_wrapper(folder_name, n, init, goal)
                        execution_times.append((init_goal, elapsed_time))
        return execution_times

    def get_execution_times(self):
        return [time[1] for time in self.execution_times]

class CamaraMapPrismRunner(CamaraMapRunner):
    def __init__(self, 
            result_folder="results/map_camara_2020_paper/prism",
            map_path='map_camara_2020_paper/map-p2cp3.json'):
        super().__init__(PrismModelgenerator, result_folder)
        self.map_generator.load_json(map_path)

    def execute_case(self, map_folder, init, goal):
        prism_filename = map_folder / f'l{init}_l{goal}.prism'
        nav_path = self.map_generator.find_shortest_path(from_node=init, to_node=goal)
        # print(nav_path)
        self.map_generator.generate_prism_model(prism_filename, nav_path)

        # Construct the PRISM command
        command = [
            "prism",
            str(prism_filename),
            "-javamaxmem", "16g",
            "-cuddmaxmem", "16g",
            "-pf", 'R{"energy"}max=? [ F stop ]',
            "-exportstrat", "stdout",
            "-const", f"INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION={len(nav_path)-1},INITIAL_CONFIGURATION=1"
        ]

        # Measure execution time
        start_time = time.perf_counter()
        self._run_subprocess(command)
    
        return time.perf_counter() - start_time

class CamaraMapPDDLRunner(CamaraMapRunner):
    def __init__(self, result_folder="results/map_camara_2020_paper/pddl", map_path='map_camara_2020_paper/map-p2cp3.json'):
        super().__init__(PrismModelgenerator, result_folder)
        self.map_generator.load_and_discretize_json(map_path)

    def execute_case(self, map_folder, init, goal):
        problem_filename = map_folder / 'problem.pddl'
        self.map_generator.generate_domain_problem_files(save_problem=True, problem_filename=problem_filename, init_goal=(init, goal))

        # Construct the OWLToPDDL command
        domain_output = map_folder / 'domain_created.pddl'
        problem_input = map_folder / 'problem.pddl'
        problem_output = map_folder / 'problem_created.pddl'
        owl_to_pddl_command = [
            "OWLToPDDL.sh",
            "--owl=owl/navigation.owl",
            "--tBox",
            "--inDomain=pddl/domain_sas.pddl",
            f"--outDomain={domain_output}",
            "--aBox",
            f"--inProblem={problem_input}",
            f"--outProblem={problem_output}",
            "--replace-output",
            "--add-num-comparisons"
        ]
        
        try:
            self._run_subprocess(owl_to_pddl_command)
        except subprocess.CalledProcessError as e:
            return self.execute_case(map_folder, init, goal)

        # run_case_wrapper the script
        command = [
            "fast-downward.py",
            domain_output,
            problem_output,
            "--search", "astar(blind())"
        ]
        
        # Start high-precision timer
        start_time = time.perf_counter()
        
        self._run_subprocess(command)
        
        # Compute and print execution time
        return time.perf_counter() - start_time
            
def box_plot(y1, y2, folder, labels=("Method A", "Method B"), title="Comparison of Execution Times", show=True):
    data = [y1, y2]
    means = [np.mean(d) for d in data]
    stds = [np.std(d) for d in data]

    fig, ax = plt.subplots(figsize=(8, 5))

    # Create box plot
    box = ax.boxplot(data, patch_artist=True, showmeans=True, labels=labels)

    # Optional: overlay mean ± std deviation markers
    x_positions = np.arange(1, len(data) + 1)
    ax.errorbar(
        x_positions,
        means,
        yerr=stds,
        fmt='o',
        color='red',
        label='Mean ± Std Dev'
    )

    # Axis labels and title
    ax.set_ylabel("Execution Time (s)")
    ax.set_title(title)
    ax.set_ylim(bottom=0)  # Start Y-axis at 0
    ax.legend()
    ax.grid(True)

    plt.savefig(Path(folder, 'box_plot.png'), format='png', dpi=300, bbox_inches='tight')

    if show:
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
    result_folder = Path("results/grid_map/", date) 
    grid_map_experiment = GridMapRunner(
        result_folder=result_folder,
        min_nodes = 10,
        max_nodes = 1000,
        nodes_interval = 10,
        nodes_skip = 0.1,
        unconnected_amount = 0.15, 
        unsafe_amount = 0.25,
        dark_amount = 0.25)
    grid_map_experiment.run_experiment(n_runs=10)
    grid_map_experiment.generate_save_plot(show=False)

    date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
    result_folder = Path("results/map_camara_2020_paper/", date) 
    
    experiment = CamaraMapPrismRunner(result_folder=result_folder / 'prism')  # You can modify n_runs if needed
    experiment.run_experiment(n_runs=10)

    experiment_pddl = CamaraMapPDDLRunner(result_folder=result_folder / 'pddl')  # You can modify n_runs if needed
    experiment_pddl.run_experiment(n_runs=10)

    box_plot(
        experiment.get_execution_times(), 
        experiment_pddl.get_execution_times(),
        labels=("Cámara et al. (2020)", "PDDL-SAS"), 
        title="Comparison of Execution Times",
        show=False,
        folder=result_folder)