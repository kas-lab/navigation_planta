from pathlib import Path
from map_generator import MapGenerator
from datetime import datetime
import subprocess
import time
import matplotlib.pyplot as plt
import numpy as np

def run(folder_name, run, n_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount):
    n_nodes_resulting = n_nodes - int(n_nodes*nodes_skip)
    map_generator = MapGenerator(
        n_nodes,
        nodes_skip, 
        unconnected_amount, 
        unsafe_amount, 
        dark_amount)
    map_generator.generate_connected_grid_map()
    
    map_folder_name = f'{n_nodes}_{nodes_skip}_{unconnected_amount}_{unsafe_amount}_{dark_amount}_{run}'
    map_folder = folder_name / map_folder_name
    if map_folder.is_dir() is False:
        map_folder.mkdir(parents=True)

    problem_filename = map_folder / 'problem.pddl'
    map_generator.generate_domain_problem_files(save_problem=True, problem_filename=problem_filename)
    
    
    map_filename = 'map.png'
    map_full_path = map_folder / map_filename
    map_generator.plot_graph(show_plot=False, save_file=True, filename=map_full_path)

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
    print(f"Total number of nodes: {n_nodes_resulting}. Execution Time: {elapsed_time:.6f} seconds")
    return (n_nodes_resulting, elapsed_time)

def runner():
    n_runs_per_n_nodes = 10
    min_nodes = 10
    max_nodes = 300
    nodes_interval = 5

    nodes_skip = 0.1 # percentage
    unconnected_amount = 0.15 # percentage
    unsafe_amount = 0.25 # percentage
    dark_amount = 0.25 # percentage

    planning_time_list = []  # List of (n_nodes_resulting, elapsed_time)

    date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
    folder_name =  Path('results', 'grid_map', date)
    if folder_name.is_dir() is False:
        folder_name.mkdir(parents=True)

    for n_nodes in range(min_nodes, max_nodes + nodes_interval, nodes_interval):
        for n in range(n_runs_per_n_nodes):
            n_nodes_resulting, elapsed_time = run(folder_name, n, n_nodes, nodes_skip, unconnected_amount, unsafe_amount, dark_amount)
            planning_time_list.append((n_nodes_resulting, elapsed_time))
    
    planning_time_csv = folder_name / 'planning_times.csv'
    planning_time_array = np.array(planning_time_list, dtype=[("nodes", "i4"), ("time", "f8")])
    # np.save("planning_times.npy", planning_time_array)
    np.savetxt(planning_time_csv, planning_time_array, delimiter=",", header="nodes,time", comments="")
    
    unique_nodes, indices = np.unique(planning_time_array["nodes"], return_inverse=True)
    mean_times = np.bincount(indices, weights=planning_time_array["time"]) / np.bincount(indices)

    # Compute standard deviation for each group
    sum_squared_diffs = np.bincount(indices, weights=(planning_time_array["time"] - mean_times[indices])**2) 
    std_dev_times = np.sqrt(sum_squared_diffs / np.bincount(indices))  # Standard deviation formula

    # Create the plot
    fig, ax = plt.subplots(figsize=(8, 5))

    # Plot mean execution times
    ax.plot(unique_nodes, mean_times, marker='o', linestyle='-', color='b', label="Avg Execution Time")

    # Add standard deviation as error bars
    ax.errorbar(unique_nodes, mean_times, yerr=std_dev_times, fmt='o', color='r', capsize=5, label="Std Dev")

    # Labels and title
    ax.set_xlabel("Number of Nodes")
    ax.set_ylabel("Mean Elapsed Time (seconds)")
    ax.set_title("Average Planning Execution Time with Std Dev")
    ax.legend()
    ax.grid(True)

    # Save the plot
    planning_time_png = folder_name / 'planning_time_avg_std.png'
    plt.savefig(planning_time_png, format='png', dpi=300, bbox_inches='tight')

    # Show only this plot
    plt.show(block=True)
    plt.close(fig)  # Ensure this figure is closed after display

if __name__ == '__main__':
    runner()