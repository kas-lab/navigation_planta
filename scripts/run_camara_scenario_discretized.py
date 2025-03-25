from map_generator import MapGenerator
import subprocess
import time
from pathlib import Path
from datetime import datetime
import numpy as np

def run(folder_name, run, init, goal):
    # n_nodes_resulting = n_nodes - int(n_nodes*nodes_skip)
    map_generator = MapGenerator()
    base_folder = Path("map_camara_2020_paper")
    map_generator.load_and_discretize_json(base_folder / "map-p2cp3.json")
    
    map_folder_name = f'wp{init}_wp{goal}_{run}'
    map_folder = folder_name / map_folder_name
    if map_folder.is_dir() is False:
        map_folder.mkdir(parents=True)

    problem_filename = map_folder / 'problem.pddl'
    map_generator.generate_domain_problem_files(save_problem=True, problem_filename=problem_filename, init_goal=(init, goal))

    domain_output = map_folder / 'domain_created.pddl'
    problem_input = map_folder / 'problem.pddl'
    problem_output = map_folder / 'problem_created.pddl'
    subprocess.run([
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
    ], capture_output=True, text=True, check=True)

    # Start high-precision timer
    start_time = time.perf_counter()

    # Run the script
    subprocess.run([
        "fast-downward.py",
        domain_output,
        problem_output,
        "--search", "astar(blind())"
    ], check=True)

    # Stop timer
    end_time = time.perf_counter()

    # Compute and print execution time
    elapsed_time = end_time - start_time
    print(f"Execution Time: {elapsed_time:.6f} seconds")
    return (f'wp{init}_wp{goal}', elapsed_time)

def runner():
    n_runs = 1
    planning_time_list = []  # List of (n_nodes_resulting, elapsed_time)

    date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
    folder_name =  Path('map_camara_2020_paper', 'discretized', date)
    if folder_name.is_dir() is False:
        folder_name.mkdir(parents=True)

    # for n_nodes in range(min_nodes, max_nodes + nodes_interval, nodes_interval):
    unreacheable_nodes = [20, 41]
    for init in range(1, 5):
        for goal in range(1, 2):
            if init != goal and init not in unreacheable_nodes and goal not in unreacheable_nodes:
                for n in range(n_runs):
                    init_goal, elapsed_time = run(folder_name, n, init, goal)
                    planning_time_list.append((init_goal, elapsed_time))
    
    planning_time_csv = folder_name / 'planning_times.csv'
    planning_time_array = np.array(planning_time_list, dtype=[("init_goal", "U15"), ("time", "f8")])
    np.savetxt(planning_time_csv, planning_time_array, delimiter=",", header="init_goal,time", comments="", fmt="%s,%.18e")
    mean = np.mean(planning_time_array["time"])
    std_dev = np.std(planning_time_array["time"])
    print(f'Mean {mean} and Std dev: {std_dev}')

if __name__ == '__main__':
    runner()
    # map_generator = MapGenerator()
    # base_folder = Path("map_camara_2020_paper")
    # map_generator.load_and_discretize_json(base_folder / "map-p2cp3.json")
    # map_generator.plot_graph(show_plot=True)

