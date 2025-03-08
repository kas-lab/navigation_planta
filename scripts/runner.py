from pathlib import Path
from map_generator import MapGenerator
from datetime import datetime


def runner():
    min_nodes = 10
    max_nodes = 50
    nodes_interval = 10

    nodes_skip = 0.1 # percentage
    unconnected_amount = 0.15 # percentage
    unsafe_amount = 0.25 # percentage
    dark_amount = 0.25 # percentage

    date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
    folder_name =  Path('results', date)
    if folder_name.is_dir() is False:
            folder_name.mkdir(parents=True)

    for n_nodes in range(min_nodes, max_nodes + nodes_interval, nodes_interval):
        map_generator = MapGenerator(
            n_nodes,
            nodes_skip, 
            unconnected_amount, 
            unsafe_amount, 
            dark_amount)
        map_generator.generate_connected_grid_map()
        map_generator.generate_domain_problem_files()
        
        map_filename = f'map_{n_nodes}_{nodes_skip}_{unconnected_amount}_{unsafe_amount}_{dark_amount}.png'
        map_full_path = folder_name / map_filename
        map_generator.plot_graph(show_plot=False, save_file=True, filename=map_full_path)


if __name__ == '__main__':
    runner()