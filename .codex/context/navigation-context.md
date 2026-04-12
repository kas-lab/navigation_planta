# Navigation Context

This file preserves stable Codex-facing context for `src/navigation_planta/`.

`AGENTS.md` is the repository policy. This context file is reference material and does not override repository rules.

## Repository Identity

- Repository root: `/home/gus/ros_workspaces/planta_ws/src/navigation_planta`
- Purpose: standalone PLANTA navigation scenario and experiment harness used by the PLANTA paper

## Primary Components

- `scripts/runner.py`: full experiment runner
- `scripts/run_grid_map_scenario.py`: grid-only runner
- `scripts/map_generator.py`: map generation, plotting, UP problem construction, and problem PDDL writing
- `scripts/prism_model_generator.py`: PRISM model generation
- `pddl/`: base domain files
- `owl/`: ontology inputs for OWL-to-PDDL

## Stable Performance Facts

- Reported planning time in the grid experiment excludes map generation, plotting, problem-file generation, and `OWLToPDDL.sh`.
- The grid generator is connected-by-construction.
- The problem file is written with `write_problem_fast()` while preserving the in-memory UP defaults for `robot_at` and `corridor`.

## Validation Pattern

- Prefer isolated timing of map generation, problem generation, OWL conversion, and Fast Downward over running the entire experiment suite.
