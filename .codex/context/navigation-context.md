# Navigation Context

This file preserves stable Codex-facing context for `src/navigation_planta/`.

`AGENTS.md` is the repository policy. This context file is reference material and does not override repository rules.

## Repository Identity

- Repository root: `/home/gus/ros_workspaces/planta_ws/src/navigation_planta`
- Purpose: standalone PLANTA navigation scenario and experiment harness used by the PLANTA paper

## Primary Components

- `scripts/runner.py`: full experiment runner
- `scripts/run_grid_map_scenario.py`: grid-only runner
- `scripts/no_adaptation.py`: shared stripped-problem generators for no-adaptation PDDL baselines
- `scripts/map_generator.py`: map generation, plotting, UP problem construction, and problem PDDL writing
- `scripts/prism_model_generator.py`: PRISM model generation
- `pddl/`: base domain files
- `owl/`: ontology inputs for OWL-to-PDDL

## Stable Performance Facts

- Reported planning time in the grid experiment excludes map generation, plotting, problem-file generation, and `OWLToPDDL.sh`.
- The grid generator is connected-by-construction.
- The problem file is written with `write_problem_fast()` while preserving the in-memory UP defaults for `robot_at` and `corridor`.

## Baseline Experiment Facts

- The standalone PDDL experiment scripts now support `--mode adaptive|no-adaptation|both`, with `adaptive` kept as the default.
- No-adaptation runs skip `OWLToPDDL.sh` entirely and call Fast Downward directly on stripped baseline domains.
- For baseline-only navigation tasks, `scripts/no_adaptation.py` provides `NoAdaptationMapGenerator`, which writes only `robot_at`, `corridor`, and the init/goal facts.
- For mission-action baselines, `scripts/no_adaptation.py` provides `MissionNoAdaptationMapGenerator`, which preserves mission waypoint and `*_done` goal predicates while omitting adaptation-specific QA requirements.
- Experiment A and B baselines reuse `pddl/domain_no_sas.pddl`.
- Experiment C baselines use `pddl/experiment_ma_scalability/domain_no_sas_ma_{2,3,4,5}.pddl`.
- Experiment D baseline uses `pddl/experiment_combined_scalability/domain_no_sas_combined.pddl`.
- In the CSV output, no-adaptation runs record `owltopddl_time=0.0` because OWL preprocessing is intentionally bypassed.

## Validation Pattern

- Prefer isolated timing of map generation, problem generation, OWL conversion, and Fast Downward over running the entire experiment suite.
