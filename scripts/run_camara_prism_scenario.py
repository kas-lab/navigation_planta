from navigation_planta.prism_model_generator import PrismModelgenerator
from navigation_planta.reporting import (
    plot_mode_summary,
    summarize_mode_records,
    write_summary_report,
)
from navigation_planta.utils import (
    ExperimentRecord,
    run_subprocess_with_memory,
    save_experiment_records_csv,
)
import subprocess
import time
from pathlib import Path
from datetime import datetime

def run(folder_name, run, init, goal):
    # n_nodes_resulting = n_nodes - int(n_nodes*nodes_skip)
    map_generator = PrismModelgenerator()
    base_folder = Path("data/map_camara_2020_paper")
    map_generator.load_json(base_folder / "map-p2cp3.json")

    map_folder_name = f'wp{init}_wp{goal}_{run}'
    map_folder = folder_name / map_folder_name
    if map_folder.is_dir() is False:
        map_folder.mkdir(parents=True)

    prism_filename = map_folder / f'l{init}_l{goal}.prism'
    nav_path = map_generator.find_shortest_path(from_node=init, to_node=goal)
    print(nav_path)
    map_generator.generate_prism_model(prism_filename, nav_path)

    strat_file = map_folder / 'strategy.txt'
    command = [
        "prism",
        str(prism_filename),
        "-pf",
        'R{"energy"}max=? [ F stop ]',
        "-exportstrat",
        f"{strat_file}:type=actions",
        "-const",
        f"INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION={len(nav_path)-1},INITIAL_CONFIGURATION=1",
    ]

    action_count = -1
    start_time = time.perf_counter()
    peak_memory = run_subprocess_with_memory(command)
    elapsed_time = time.perf_counter() - start_time

    if strat_file.exists():
        action_count = map_generator.count_plan_actions_from_strategy(
            strat_file.read_text(),
            nav_path,
            initial_battery=32560,
            initial_config=1,
        )
        strat_file.unlink()

    print(f"Execution Time: {elapsed_time:.6f} seconds  actions={action_count}  mem={peak_memory:.1f}MB")
    return ExperimentRecord(
        mode='prism',
        x_value=f'wp{init}_wp{goal}',
        planning_time=elapsed_time,
        action_count=action_count,
        peak_memory=peak_memory,
    )


def runner(out_dir: Path | None = None) -> Path:
    n_runs = 1
    planning_time_list = []

    if out_dir is not None:
        folder_name = out_dir
    else:
        date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
        folder_name = Path('results', 'map_camara_2020_paper', 'prism', date)
    if folder_name.is_dir() is False:
        folder_name.mkdir(parents=True)

    # for n_nodes in range(min_nodes, max_nodes + nodes_interval,
    # nodes_interval):
    unreacheable_nodes = [20, 41]
    for init in range(1, 59):
        for goal in range(1, 59):
            if init != goal and init not in unreacheable_nodes and goal not in unreacheable_nodes:
                for n in range(n_runs):
                    planning_time_list.append(run(folder_name, n, init, goal))

    planning_time_csv = folder_name / 'planning_times.csv'
    save_experiment_records_csv(
        planning_time_csv,
        planning_time_list,
        x_name='init_goal',
        time_name='time',
    )
    summary_lines = summarize_mode_records(
        planning_time_list,
        {'prism': 'Cámara et al. (2020)'},
    )
    for line in summary_lines:
        print(line)
    write_summary_report(folder_name, summary_lines)

    plot_mode_summary(
        folder_name, planning_time_list,
        {'prism': 'Cámara et al. (2020)'},
        filename='camara_prism_summary.png')
    return planning_time_csv


if __name__ == '__main__':
    runner()
