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

- `scripts/runner.py`: main experiment entrypoint; runs grid, PRISM, and PDDL comparison experiments
- `scripts/run_grid_map_scenario.py`: grid-only experiment entrypoint
- `scripts/map_generator.py`: random/grid map generation, plotting, and UP/PDDL problem generation
- `scripts/prism_model_generator.py`: PRISM model generation for the Cámara-style comparison
- `pddl/`: base domain files used by OWL-to-PDDL
- `owl/`: ontology inputs for OWL-to-PDDL
- `results/`: generated experiment outputs
- `map_camara_2020_paper/`: navigation map assets used for the comparison scenario

## Environment And Common Commands

Build image locally:

```bash
docker build -t navigation_planta .
```

Open an interactive container with the repo mounted:

```bash
docker run --rm -it --name navigation_planta -v $PWD/src/navigation_planta:/navigation_pddl_tomasys/ -v /etc/localtime:/etc/localtime:ro navigation_planta:latest bash
```

Run the full experiment suite:

```bash
python scripts/runner.py
```

Run only the grid experiment:

```bash
python scripts/run_grid_map_scenario.py
```

## Timing Rule

- The grid-scenario wall-clock time is not the same as the reported planning time.
- In `scripts/runner.py`, `GridMapRunner.execute_case()` starts timing only around the `fast-downward.py` call.
- Map generation, problem-file writing, plotting, and `OWLToPDDL.sh` happen before that timer starts.
- Do not infer end-to-end runtime from the CSV planning times alone.

## Grid Experiment Performance Notes

- `scripts/map_generator.py` now generates connected grid maps by construction instead of retrying until a connected sample appears.
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
- Do not run `scripts/runner.py` just to profile one suspected hotspot.
- When validating planner-path changes, check both:
  - that `OWLToPDDL.sh` succeeds
  - that `fast-downward.py` still finds a solution on a representative case
