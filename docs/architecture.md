# Navigation PLANTA Architecture

This note summarizes the current repository structure after the refactor phases.

## Canonical Entrypoints

Use the standalone scripts in `scripts/` as the supported experiment entrypoints:

- `run_grid_map_scenario.py`
- `run_camara_scenario.py`
- `run_camara_scenario_discretized.py`
- `run_camara_prism_scenario.py`
- `run_fd_scale_scenario.py`
- `run_corridor_type_scale_scenario.py`
- `run_mission_action_scale_scenario.py`
- `run_combined_scenario.py`
- `run_all_experiments.py`
- `benchmark_search_strategies.py`

The removed `scripts/runner.py` is no longer part of the execution model.

## Package Structure

The reusable code lives in `navigation_planta/`:

- `experiment_runner.py`: shared sweep orchestration and bounded retry flow
- `reporting.py`: numeric sweep plots, categorical summaries, memory summaries, comparison plots, and summary reports
- `utils.py`: planner execution helpers, CSV serialization, and plan-length parsing
- `scenario_variants.py`: package-level adaptive and no-adaptation scenario variants
- `no_adaptation.py`: baseline map generator behavior without adaptive predicates
- `map_generator.py`: thin facade that composes the focused map/PDDL mixins
- `graph_generation.py`: graph generation and pathfinding helpers
- `map_io.py`: JSON loading, discretization, and graph plotting
- `pddl_builder.py`: Unified Planning domain/problem construction and serialization
- `prism_model_generator.py`: PRISM model generation and strategy simulation

## Experiment Flow

The PDDL experiment scripts now follow the same structure:

1. define experiment constants and CLI options
2. build or load a scenario generator
3. serialize a base PDDL problem
4. optionally run `OWLToPDDL.sh` for adaptive mode
5. run Fast Downward and collect planning time, action count, and peak memory
6. save `planning_times.csv`
7. generate standardized plots and, for Cámara-style runs, `summary.txt`

`run_all_experiments.py` delegates to the same entrypoints and explicitly calls
`reporting.plot_strategy_line_comparison()` for numeric sweeps and
`reporting.plot_strategy_box_comparison()` for Cámara-style comparisons.
`scripts/regenerate_comparison_plots.py` rebuilds the same comparison figures
from an existing `results/all_experiments/<timestamp>` folder by probing only
the expected `planning_times.csv` locations.

## Scenario Extension Model

Scenario-specific domain behavior belongs in `scenario_variants.py`, not in CLI scripts.

- corridor-type scaling composes corridor requirement features
- mission-action scaling composes mission-goal features
- the combined scenario composes both feature families
- no-adaptation baselines reuse the same scenario concepts beside their adaptive counterparts

## Adding A New Experiment

For a new sweep:

1. add or reuse a scenario generator in `navigation_planta/`
2. create a thin script in `scripts/` that defines constants and CLI parsing
3. use `run_sweep_experiment()` from `experiment_runner.py`
4. use `reporting.py` helpers for plots and summaries
5. add focused tests for any new scenario or reporting behavior
