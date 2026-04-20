# AGENTS.md

## Project Scope

This file applies to work inside `src/navigation_planta/`.

This repository contains the standalone PLANTA navigation scenario used in the PLANTA paper experiments, including:

- grid-map scalability experiments
- the replicated Cámara-style comparison flows
- map and PRISM model generation
- OWL-to-PDDL and Fast Downward planning integration

This file is the repository policy. Stable repo facts that are useful across tasks belong in `.codex/context/navigation-context.md`.

## Repository Layout

- `scripts/run_camara_scenario.py`: Camara map experiment entrypoint
- `scripts/run_camara_scenario_discretized.py`: discretized Camara experiment entrypoint
- `scripts/run_camara_prism_scenario.py`: PRISM baseline entrypoint
- `scripts/run_grid_map_scenario.py`: grid scalability experiment entrypoint
- `scripts/run_fd_scale_scenario.py`: FunctionDesign scalability entrypoint
- `scripts/run_corridor_type_scale_scenario.py`: corridor-type scalability entrypoint
- `scripts/run_mission_action_scale_scenario.py`: mission-action scalability entrypoint
- `scripts/run_combined_scenario.py`: combined scalability entrypoint
- `scripts/run_all_experiments.py`: full suite entrypoint
- `navigation_planta/experiment_runner.py`: shared experiment orchestration
- `navigation_planta/reporting.py`: shared plotting, comparison, and summary helpers
- `navigation_planta/scenario_variants.py`: package-level scenario variants
- `navigation_planta/map_generator.py`: facade over graph generation, map I/O, and PDDL generation
- `navigation_planta/prism_model_generator.py`: PRISM model generation for the Cámara-style comparison
- `pddl/`: base domain files used by OWL-to-PDDL
- `owl/`: ontology inputs for OWL-to-PDDL
- `results/`: generated experiment outputs
- `data/map_camara_2020_paper/`: navigation map assets used for the comparison scenario

## Environment And Common Commands

Build image locally:

```bash
docker build -t navigation_planta .
```

Open an interactive container with the repo mounted:

```bash
docker run --rm -it --name navigation_planta -v $PWD/src/navigation_planta:/navigation_planta/ -v /etc/localtime:/etc/localtime:ro navigation_planta:latest bash
```

Run the full experiment suite:

```bash
python scripts/run_all_experiments.py
```

Run only the grid experiment:

```bash
python scripts/run_grid_map_scenario.py
```

Run a no-adaptation baseline for a PDDL experiment:

```bash
python scripts/run_grid_map_scenario.py --mode no-adaptation
```

Run both adaptive and no-adaptation variants for comparison:

```bash
python scripts/run_grid_map_scenario.py --mode both
```

The standalone PDDL experiment entrypoints follow the same mode convention:

- `--mode adaptive`: current OWL-to-PDDL PLANTA flow
- `--mode no-adaptation`: direct Fast Downward baseline without OWL preprocessing
- `--mode both`: run both variants and save a `mode` column in the CSV output

## Timing Rule

- The grid-scenario wall-clock time is not the same as the reported planning time.
- In the current experiment scripts, timing starts only around the `fast-downward.py` call.
- Map generation, problem-file writing, plotting, and `OWLToPDDL.sh` happen before that timer starts.
- Do not infer end-to-end runtime from the CSV planning times alone.

## Grid Experiment Performance Notes

- `navigation_planta/map_generator.py` now generates connected grid maps by construction instead of retrying until a connected sample appears.
- `generate_grid_graph()` keeps a connected subset of cells, adds only local 4-neighbor edges, and removes only edges outside a spanning tree.
- `generate_domain_problem_files()` uses `write_problem_fast()` for the problem file because `unified_planning`'s generic `PDDLWriter.write_problem()` is too slow for this scenario.

## Serialization Rule

- Keep `default_initial_value=False` for `robot_at` and `corridor` in the in-memory UP problem.
- When writing the problem PDDL for this repo, prefer `write_problem_fast()` because it serializes only explicit initial values.
- This fast writer is validated for the current navigation problem shape used by the experiments; if the problem gains metrics, timed effects, HTN features, or other richer PDDL constructs, review the writer before reusing it.

## Validation Guidance

- For performance debugging, prefer isolated measurements of:
  - connected map generation
  - problem-file generation
  - `OWLToPDDL.sh`
  - `fast-downward.py`
- Do not run `scripts/run_all_experiments.py` just to profile one suspected hotspot.
- When validating planner-path changes, check both:
  - that `OWLToPDDL.sh` succeeds
  - that `fast-downward.py` still finds a solution on a representative case
