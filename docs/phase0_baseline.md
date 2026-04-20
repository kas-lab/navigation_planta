# Phase 0 Baseline

This document captures the current `navigation_planta` experiment entrypoints, their expected outputs, and a lightweight validation checklist to run after each refactor phase.

The goal is not to exhaustively benchmark the suite on every change. The goal is to keep a stable reference for:

- canonical commands
- output folder layout
- CSV schemas
- expected plot filenames
- a small manual validation matrix

## Representative Validation Matrix

Use these commands as representative checks after behavior-preserving refactors. They do not need to be run on every commit, but they should remain valid.

| Script | Representative command | Primary output root | Notes |
|---|---|---|---|
| `scripts/run_grid_map_scenario.py` | `python scripts/run_grid_map_scenario.py --mode adaptive --runs 1 --search 'astar(blind())'` | `results/grid_map/<date>/adaptive/` | Numeric node sweep, adaptive flow |
| `scripts/run_grid_map_scenario.py` | `python scripts/run_grid_map_scenario.py --mode no-adaptation --runs 1 --search 'astar(blind())'` | `results/grid_map/<date>/no-adaptation/` | Numeric node sweep, baseline flow |
| `scripts/run_camara_scenario.py` | `python scripts/run_camara_scenario.py --mode adaptive --runs 1` | `results/map_camara_2020_paper/<date>/` | Original Cámara PDDL experiment |
| `scripts/run_camara_scenario_discretized.py` | `python scripts/run_camara_scenario_discretized.py --mode adaptive --runs 1` | `results/map_camara_2020_paper/discretized/<date>/adaptive/` | Discretized Cámara PDDL experiment |
| `scripts/run_camara_prism_scenario.py` | `python scripts/run_camara_prism_scenario.py` | `results/map_camara_2020_paper/prism/<date>/` | PRISM baseline |
| `scripts/run_fd_scale_scenario.py` | `python scripts/run_fd_scale_scenario.py --mode adaptive --runs 1 --search 'astar(blind())'` | `results/scalability_fd/<date>/adaptive/` | Experiment A |
| `scripts/run_corridor_type_scale_scenario.py` | `python scripts/run_corridor_type_scale_scenario.py --mode adaptive --runs 1 --search 'astar(blind())'` | `results/scalability_ct/<date>/adaptive/` | Experiment B |
| `scripts/run_mission_action_scale_scenario.py` | `python scripts/run_mission_action_scale_scenario.py --mode adaptive --runs 1 --search 'astar(blind())'` | `results/scalability_ma/<date>/adaptive/` | Experiment C |
| `scripts/run_combined_scenario.py` | `python scripts/run_combined_scenario.py --mode adaptive --runs 1 --search 'astar(blind())'` | `results/scalability_combined/<date>/adaptive/` | Experiment D |
| `scripts/run_all_experiments.py` | `python scripts/run_all_experiments.py --runs 1 --search 'astar(blind())'` | `results/all_experiments/<date>/` | Aggregates the suite |
| `scripts/benchmark_search_strategies.py` | `python scripts/benchmark_search_strategies.py --nodes 10,20 --runs 1 --timeout 60` | `results/benchmark_search/<date>/` | Strategy benchmark on Experiment D |

## Current Output Reference

### `scripts/run_grid_map_scenario.py`

- Default results root: `results/grid_map/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `nodes,time,action_count,peak_memory`
  - both modes: `mode,nodes,time,action_count,peak_memory`
- Plot files:
  - `planning_time_avg_std.png`
  - `peak_memory_boxplot.png`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`
  - `--show-plot`

### `scripts/run_camara_scenario.py`

- Default results root: `results/map_camara_2020_paper/<date>/`
- Case folders: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `init_goal,time,action_count,peak_memory`
  - both modes: `mode,init_goal,time,action_count,peak_memory`
- Plot files:
  - `camara_summary.png`
  - `summary.txt`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`

### `scripts/run_camara_scenario_discretized.py`

- Default results root: `results/map_camara_2020_paper/discretized/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `init_goal,time,action_count,peak_memory`
  - both modes: `mode,init_goal,time,action_count,peak_memory`
- Plot files:
  - `camara_discretized_summary.png`
  - `summary.txt`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`

### `scripts/run_camara_prism_scenario.py`

- Default results root: `results/map_camara_2020_paper/prism/<date>/`
- CSV: `planning_times.csv`
- CSV columns:
  - `init_goal,time,action_count,peak_memory`
- Plot files:
  - `camara_prism_summary.png`
  - `summary.txt`
- Main CLI args:
  - no scenario-specific CLI options today

### `scripts/run_fd_scale_scenario.py`

- Default results root: `results/scalability_fd/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `n_fd,owltopddl_time,planning_time,action_count,peak_memory`
  - both modes: `mode,n_fd,owltopddl_time,planning_time,action_count,peak_memory`
- Plot files:
  - `planning_time_fd_scale.png`
  - `peak_memory_boxplot.png`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`

### `scripts/run_corridor_type_scale_scenario.py`

- Default results root: `results/scalability_ct/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `k,owltopddl_time,planning_time,action_count,peak_memory`
  - both modes: `mode,k,owltopddl_time,planning_time,action_count,peak_memory`
- Plot files:
  - `planning_time_ct_scale.png`
  - `peak_memory_boxplot.png`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`

### `scripts/run_mission_action_scale_scenario.py`

- Default results root: `results/scalability_ma/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `m,owltopddl_time,planning_time,action_count,peak_memory`
  - both modes: `mode,m,owltopddl_time,planning_time,action_count,peak_memory`
- Plot files:
  - `planning_time_ma_scale.png`
  - `peak_memory_boxplot.png`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`

### `scripts/run_combined_scenario.py`

- Default results root: `results/scalability_combined/<date>/`
- Per-mode folder: `<root>/<mode>/`
- CSV: `planning_times.csv`
- CSV columns:
  - single mode: `nodes,owltopddl_time,planning_time,action_count,peak_memory`
  - both modes: `mode,nodes,owltopddl_time,planning_time,action_count,peak_memory`
- Plot files:
  - `planning_time_combined.png`
  - `peak_memory_boxplot.png`
- Main CLI args:
  - `--mode adaptive|no-adaptation|both`
  - `--runs N`
  - `--search SEARCH`
  - `--min-nodes N`
  - `--max-nodes N`

### `scripts/run_all_experiments.py`

- Default results root: `results/all_experiments/<date>/`
- Per-strategy folder: `<root>/<sanitized-search>/`
- Comparison folder: `<root>/comparison/`
- Outputs:
  - delegated experiment CSVs and plots under each strategy folder
  - comparison plots named `comparison_<experiment><suffix>.png`
- Main CLI args:
  - `--runs N`
  - `--skip NAME [NAME ...]`
  - `--search SEARCH [SEARCH ...]`

### `scripts/benchmark_search_strategies.py`

- Default results root: `results/benchmark_search/<date>/`
- CSV: `benchmark_results.csv`
- CSV columns:
  - `strategy,n_nodes,planning_time,plan_length`
- Plot files:
  - `benchmark_comparison.png`
- Main CLI args:
  - `--nodes CSV`
  - `--runs N`
  - `--timeout SECONDS`

## Current Shared Behaviors Worth Protecting

These are good candidates for automated tests because later refactors are likely to touch them:

- `navigation_planta.utils.count_plan_actions()`
- `navigation_planta.reporting.plot_mode_summary()`
- `navigation_planta.reporting.plot_memory_summary()`
- `navigation_planta.map_generator.MapGenerator.write_problem_fast()`
- `navigation_planta.map_generator.MapGenerator.generate_domain_problem_files()`

## Phase Review Checklist

Run this checklist after each behavior-preserving refactor phase:

1. Do the representative commands above still exist and parse their current CLI arguments?
2. Does each entrypoint still write results under the documented root folder?
3. Is `planning_times.csv` or the benchmark CSV still emitted with the expected columns?
4. Are the documented plot files still created?
5. Do the Cámara scripts still use the summary plot functions rather than custom one-off plotting?
6. Does the generated PDDL still contain:
   - typed waypoint objects
   - explicit corridor facts
   - an initial robot location
   - at least one goal
7. If a phase changes an output schema intentionally, was this document updated in the same change?

## Notes On Local Validation

- Some tests depend on the local Unified Planning stack importing cleanly.
- If the host Python environment has dependency conflicts, use the project Docker image for end-to-end validation.
- The baseline utilities and plot tests are intended to remain runnable even when the full external planning toolchain is not available.
