#!/usr/bin/env python3

# Copyright 2025 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Benchmark Fast Downward search strategies on the combined scenario (Experiment D).

Generates one map + runs OWLToPDDL once per (n_nodes, run_id) trial, then
times every strategy on the same domain_created.pddl / problem_created.pddl.
This isolates planner performance from map randomness.

All candidate strategies must support axioms (derived predicates).

Usage:
    python scripts/benchmark_search_strategies.py [options]

    --nodes   Comma-separated list of map sizes to test (default: 10,20,30,40,50)
    --runs    Trials per map size (default: 3)
    --timeout Per-strategy time limit in seconds (default: 60)
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent

sys.path.insert(0, str(SCRIPT_DIR))
from run_combined_scenario import (
    CombinedMapGenerator,
    OWL_FILE,
    ADAPTIVE_DOMAIN_FILE,
    NODES_SKIP,
    UNCONNECTED_AMOUNT,
    UNSAFE_AMOUNT,
    DARK_AMOUNT,
)

# Strategies to compare — all support axioms (derived predicates).
# Grouped by heuristic cost: cheap evaluators first, expensive last.
# ff/add proved too slow (>60s on n=10) due to axiom-layer relaxation overhead.
STRATEGIES = [
    # --- blind: no heuristic computation at all ---
    ('astar+blind',              'astar(blind())'),
    ('lazy_greedy+blind',        'lazy_greedy([blind()])'),
    ('eager_greedy+blind',       'eager_greedy([blind()])'),

    # --- goalcount: O(goals) per node, no relaxation ---
    # preferred=[goalcount()] marks operators that satisfy ≥1 goal as preferred
    ('astar+goalcount',          'astar(goalcount())'),
    ('lazy_greedy+goalcount',    'lazy_greedy([goalcount()])'),
    ('lazy_greedy+gc+pref',      'lazy_greedy([goalcount()], preferred=[goalcount()])'),
    ('eager_greedy+goalcount',   'eager_greedy([goalcount()])'),
    ('eager_greedy+gc+pref',     'eager_greedy([goalcount()], preferred=[goalcount()])'),

    # --- weighted A* with goalcount: satisficing but bounded-suboptimal ---
    # w>1 makes it prefer h over g, trading plan quality for speed
    ('wastar+gc_w2',             'lazy_wastar([goalcount()], w=2)'),
    ('wastar+gc_w5',             'lazy_wastar([goalcount()], w=5)'),
    ('wastar+gc_w10',            'lazy_wastar([goalcount()], w=10)'),
    ('wastar+gc+pref_w5',        'lazy_wastar([goalcount()], preferred=[goalcount()], w=5)'),

    # --- multi-heuristic: goalcount for guidance, blind as tiebreaker ---
    ('lazy_greedy+gc+blind',     'lazy_greedy([goalcount(), blind()])'),
    ('eager_greedy+gc+blind',    'eager_greedy([goalcount(), blind()])'),

    # --- known timeouts (kept commented for reference) ---
    # ('astar+hmax',             'astar(hmax())'),           # timeout
    # ('lazy_greedy+hmax',       'lazy_greedy([hmax()])'),   # timeout
    # ('astar+cg',               'astar(cg())'),             # timeout
    # ('lazy_greedy+cg',         'lazy_greedy([cg()])'),     # timeout
    # ('astar+ff',               'astar(ff())'),             # timeout
    # ('lazy_greedy+ff+pref',    'lazy_greedy([ff()], preferred=[ff()])'),  # timeout
    # ('lazy_greedy+add+pref',   'lazy_greedy([add()], preferred=[add()])'),# timeout
]

TIMEOUT_SENTINEL = float('inf')
NO_PLAN = -1


def count_plan_actions(plan_file: Path) -> int:
    """Count actions in an FD plan file, ignoring comment/cost lines.

    FD writes plans to {plan_file}.1, .2, ... — take the last (best for
    satisficing search). Returns NO_PLAN (-1) if no plan file is found.
    """
    candidates = sorted(plan_file.parent.glob(plan_file.name + '.*'))
    target = candidates[-1] if candidates else (plan_file if plan_file.exists() else None)
    if target is None:
        return NO_PLAN
    lines = target.read_text().splitlines()
    return sum(1 for line in lines if line.strip() and not line.startswith(';'))


def run_fd(
        domain: Path, problem: Path, search: str, timeout: float, plan_file: Path,
) -> tuple:
    """Run fast-downward.py; return (wall-clock time, plan_length) or sentinels.

    Uses start_new_session=True so fast-downward.py and the C++ binary it spawns
    share a process group. On timeout the entire group is killed with SIGKILL.
    plan_file is passed as a driver option (before domain/problem) so FD writes
    the plan to {plan_file}.1 regardless of search algorithm.
    """
    cmd = [
        'fast-downward.py',
        '--plan-file', str(plan_file),
        str(domain),
        str(problem),
        '--search', search,
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    t0 = time.perf_counter()
    try:
        proc.wait(timeout=timeout)
        if proc.returncode != 0:
            return TIMEOUT_SENTINEL, NO_PLAN
        return time.perf_counter() - t0, count_plan_actions(plan_file)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait()
        return TIMEOUT_SENTINEL, NO_PLAN


def prepare_trial(folder: Path, run_id: int, n_nodes: int):
    """Generate map, write problem.pddl, run OWLToPDDL.

    Returns (domain_created, problem_created) paths, or raises on failure.
    """
    trial_folder = folder / f'n{n_nodes}_run{run_id}'
    trial_folder.mkdir(parents=True, exist_ok=True)

    mg = CombinedMapGenerator(n_nodes, NODES_SKIP, UNCONNECTED_AMOUNT,
                               UNSAFE_AMOUNT, DARK_AMOUNT)
    mg.generate_connected_grid_map()

    problem_file = trial_folder / 'problem.pddl'
    mg.generate_domain_problem_files(save_problem=True, problem_filename=str(problem_file))

    domain_out = trial_folder / 'domain_created.pddl'
    problem_out = trial_folder / 'problem_created.pddl'

    subprocess.run(
        [
            'OWLToPDDL.sh',
            f'--owl={OWL_FILE}',
            '--tBox',
            f'--inDomain={ADAPTIVE_DOMAIN_FILE}',
            f'--outDomain={domain_out}',
            '--aBox',
            f'--inProblem={problem_file}',
            f'--outProblem={problem_out}',
            '--replace-output',
            '--add-num-comparisons',
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return domain_out, problem_out


def benchmark(node_sizes, n_runs: int, timeout: float, out_folder: Path):
    records = []  # (label, n_nodes, planning_time, plan_length)

    for n_nodes in node_sizes:
        for run_id in range(n_runs):
            print(f'\n--- n={n_nodes}  run={run_id} ---')
            try:
                domain_out, problem_out = prepare_trial(out_folder, run_id, n_nodes)
            except subprocess.CalledProcessError as e:
                print(f'  OWLToPDDL failed, skipping: {e.stderr[:120]}')
                continue

            trial_folder = out_folder / f'n{n_nodes}_run{run_id}'
            for label, search in STRATEGIES:
                plan_file = trial_folder / f'plan_{label}'
                t, plan_len = run_fd(domain_out, problem_out, search, timeout, plan_file)
                timed_out = t == TIMEOUT_SENTINEL
                time_display = f'>{timeout:.0f}s (timeout)' if timed_out else f'{t:.4f}s'
                plan_display = 'n/a' if plan_len == NO_PLAN else str(plan_len)
                print(f'  {label:<28s}  {time_display}  plan={plan_display}')
                records.append((label, n_nodes, t if not timed_out else timeout, plan_len))

    return records


def save_results(records, out_folder: Path, timeout: float):
    csv_path = out_folder / 'benchmark_results.csv'
    with open(csv_path, 'w') as f:
        f.write('strategy,n_nodes,planning_time,plan_length\n')
        for label, n, t, plan_len in records:
            f.write(f'{label},{n},{t:.6f},{plan_len}\n')
    print(f'\nResults saved to {csv_path}')

    labels = [s[0] for s in STRATEGIES]
    unique_nodes = sorted(set(r[1] for r in records))

    # Summary table: mean time and modal plan length per (strategy, n_nodes)
    col_w = max(9, max(len(str(n)) + 7 for n in unique_nodes))
    header_cols = '  '.join(f'n={n:4d}' for n in unique_nodes)
    print(f'\n{"Strategy":<28}  {header_cols}')
    print('-' * (30 + (col_w + 2) * len(unique_nodes)))
    for label in labels:
        row_vals = []
        for n in unique_nodes:
            subset = [(r[2], r[3]) for r in records if r[0] == label and r[1] == n]
            if subset:
                mean_t = np.mean([s[0] for s in subset])
                lengths = [s[1] for s in subset if s[1] != NO_PLAN]
                plan_str = f'/{int(np.median(lengths))}' if lengths else '/n/a'
                row_vals.append(f'{mean_t:5.2f}s{plan_str}')
            else:
                row_vals.append('      -')
        print(f'{label:<28}  ' + '  '.join(row_vals))

    colors = plt.get_cmap('tab10').colors  # type: ignore[attr-defined]

    # Plot 1: planning time
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 6))
    for i, (label, _) in enumerate(STRATEGIES):
        xs, ys, errs = [], [], []
        for n in unique_nodes:
            times = [r[2] for r in records if r[0] == label and r[1] == n]
            if times:
                xs.append(n)
                ys.append(np.mean(times))
                errs.append(np.std(times))
        if xs:
            ax1.errorbar(xs, ys, yerr=errs, marker='o', markersize=4,
                         label=label, color=colors[i % len(colors)], capsize=3)

    ax1.axhline(timeout, color='red', linestyle='--', linewidth=0.8,
                label=f'timeout ({timeout:.0f}s)')
    ax1.set_xlabel('Effective Map Size (nodes)')
    ax1.set_ylabel('Mean Planning Time (seconds)')
    ax1.set_title('Planning Time by Strategy')
    ax1.legend(fontsize=7)
    ax1.grid(True)

    # Plot 2: plan length (quality) — only for strategies that produced plans
    for i, (label, _) in enumerate(STRATEGIES):
        xs, ys = [], []
        for n in unique_nodes:
            lengths = [r[3] for r in records
                       if r[0] == label and r[1] == n and r[3] != NO_PLAN]
            if lengths:
                xs.append(n)
                ys.append(np.median(lengths))
        if xs:
            ax2.plot(xs, ys, marker='o', markersize=4, linestyle='-',
                     label=label, color=colors[i % len(colors)])

    ax2.set_xlabel('Effective Map Size (nodes)')
    ax2.set_ylabel('Median Plan Length (actions)')
    ax2.set_title('Plan Quality by Strategy')
    ax2.legend(fontsize=7)
    ax2.grid(True)

    fig.suptitle('Search Strategy Comparison — Combined Scenario (Experiment D)')
    plot_path = out_folder / 'benchmark_comparison.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'Plot saved to {plot_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Benchmark FD search strategies on the combined scenario.')
    parser.add_argument(
        '--nodes', default='10,20,30,40,50',
        help='Comma-separated map sizes (default: 10,20,30,40,50)')
    parser.add_argument(
        '--runs', type=int, default=3,
        help='Trials per map size (default: 3)')
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='Per-strategy time limit in seconds (default: 60)')
    args = parser.parse_args()

    node_sizes = [int(x.strip()) for x in args.nodes.split(',')]
    date = datetime.now().strftime('%d-%b-%Y-%H-%M-%S')
    out_folder = REPO_ROOT / 'results' / 'benchmark_search' / date
    out_folder.mkdir(parents=True, exist_ok=True)

    print(f'Benchmarking {len(STRATEGIES)} strategies on n={node_sizes}, '
          f'{args.runs} run(s) each, timeout={args.timeout}s')
    print(f'Output: {out_folder}\n')

    records = benchmark(node_sizes, args.runs, args.timeout, out_folder)
    save_results(records, out_folder, args.timeout)


if __name__ == '__main__':
    main()
