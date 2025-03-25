from prism_model_generator import PrismModelgenerator
import subprocess
import time
from pathlib import Path
from datetime import datetime
import numpy as np

def run(folder_name, run, init, goal):
    # n_nodes_resulting = n_nodes - int(n_nodes*nodes_skip)
    map_generator = PrismModelgenerator()
    base_folder = Path("map_camara_2020_paper")
    map_generator.load_json(base_folder / "map-p2cp3.json")
    
    map_folder_name = f'wp{init}_wp{goal}_{run}'
    map_folder = folder_name / map_folder_name
    if map_folder.is_dir() is False:
        map_folder.mkdir(parents=True)

    prism_filename = map_folder / f'l{init}_l{goal}.prism'
    nav_path = map_generator.find_shortest_path(from_node=init, to_node=goal)
    print(nav_path)
    map_generator.generate_prism_model(prism_filename, nav_path)

    # Define the command as a list
    command = [
        "prism", 
        prism_filename, 
        "-pf", 'R{"energy"}min=? [ F stop ]', 
        "-exportstrat", "stdout", 
        "-const", "INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION=7,INITIAL_CONFIGURATION=1"
    ]

    # Start high-precision timer
    start_time = time.perf_counter()
    # Run the command
    try:
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("Output:", result.stdout)
        print("Errors:", result.stderr)
    except subprocess.CalledProcessError as e:
        print("An error occurred:", e)
        print("Return code:", e.returncode)
        print("Output:", e.output)
        print("Errors:", e.stderr)

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
    folder_name =  Path('map_camara_2020_paper', 'prism', date)
    if folder_name.is_dir() is False:
        folder_name.mkdir(parents=True)

    # for n_nodes in range(min_nodes, max_nodes + nodes_interval, nodes_interval):
    unreacheable_nodes = [20, 41]
    for init in range(1, 59):
        for goal in range(1, 59):
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