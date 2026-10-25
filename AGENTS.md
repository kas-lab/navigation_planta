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

- `scripts/run_camara_scenario_discretized.py`: discretized Camara experiment entrypoint
- `scripts/run_camara_prism_scenario.py`: PRISM baseline entrypoint
- `scripts/run_grid_map_scenario.py`: grid scalability experiment entrypoint
- `scripts/run_fd_scale_scenario.py`: FunctionDesign scalability entrypoint
- `scripts/run_corridor_type_scale_scenario.py`: corridor-type scalability entrypoint
- `scripts/run_mission_action_scale_scenario.py`: mission-action scalability entrypoint
- `scripts/run_combined_scenario.py`: combined scalability entrypoint
- `scripts/run_all_experiments.py`: full suite entrypoint
- `scripts/helpers/experiment_runner.py`: shared experiment orchestration
- `scripts/helpers/reporting.py`: shared plotting, comparison, and summary helpers
- `scripts/helpers/scenario_variants.py`: shared scenario variants
- `scripts/helpers/map_generator.py`: facade over graph generation, map I/O, and PDDL generation
- `scripts/helpers/prism_model_generator.py`: PRISM model generation for the Cámara-style comparison
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
docker run --rm -it -v "$PWD":/navigation_planta -w /navigation_planta navigation_planta:latest bash
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

## Running Tests

Run tests in a container from the `navigation_planta:latest` image (built from `Dockerfile`; an image is usually already on disk, check with `docker image ls navigation_planta`). The host Python has no networkx/psutil, so don't run tests there. The image has every dependency, including PRISM, Fast Downward and pytest, so no tests get skipped. From `src/navigation_planta/`:

```bash
docker run --rm -v "$PWD":/navigation_planta -w /navigation_planta navigation_planta:latest \
  python -m pytest -q tests
```

The code runs directly from `scripts/`, without installing the repository. Mounting the checkout over `/navigation_planta` picks up local changes without a rebuild. Rebuild the image when `Dockerfile` or `requirements.txt` changes.

## Experiment And Analysis Workflows

- The paper uses the discretized Cámara PDDL runner and the PRISM runner; there is no non-discretized Cámara entrypoint.
- The default full suite uses 10 repetitions and both A* with blind and eager greedy with goal-count heuristics.
- `scripts/generate_paper_artifacts.py` is the canonical publication exporter; it validates full paper coverage and uses sample SD.
- The suite generates comparison PNGs through `scripts/helpers/reporting.py`. `scripts/regenerate_comparison_plots.py` rebuilds those same diagnostic plots; the publication exporter produces a distinct plot set, not identical replacements.
- PRISM models and PDDL problems are generated in result directories; do not add static generated examples to tracked inputs.

## Navigation Comparison Safety Semantics

- Decision: retain the current PLANTA and PRISM safety formulations. Their differences are intentional, not pending implementation work.
- PLANTA's PDDL generator assigns a safety requirement of `0.8` to unsafe corridors. PRISM filters movement commands for darkness compatibility but models collision risk using corridor- and configuration-specific hitrates, not an equivalent hard unsafe-corridor threshold.
- This distinction follows the original Cámara et al. (SEAMS 2020) approach's separation of darkness-related non-navigability and probabilistic collision risk. The user confirms that the hitrate data were obtained from the original authors.
- Do not add an unsafe/accuracy movement filter to PRISM merely to match PLANTA unless the user explicitly requests a change in comparison semantics.
- The reported PRISM comparison optimizes remaining battery; do not conflate including collision probabilities with optimizing safety or enforcing a hard safety threshold.
- Describe the comparison as using different safety formulations, not identical feasible movements or equivalent safety guarantees. The manuscript needs to disclose this explicitly; do not silently edit it or claim that this caveat is already documented.

## Timing Rule

- The grid-scenario wall-clock time is not the same as the reported planning time.
- In the current experiment scripts, timing starts only around the `fast-downward.py` call.
- Map generation, problem-file writing, plotting, and `OWLToPDDL.sh` happen before that timer starts.
- Do not infer end-to-end runtime from the CSV planning times alone.

## Grid Experiment Performance Notes

- `scripts/helpers/map_generator.py` now generates connected grid maps by construction instead of retrying until a connected sample appears.
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
