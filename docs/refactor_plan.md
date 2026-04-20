# Navigation PLANTA Refactor Plan

This document is a phased implementation plan to improve readability, maintainability, and duplication in `src/navigation_planta/` without changing experiment intent.

The phases are meant to be addressed independently. Earlier phases reduce risk for later ones, but each phase should still be reviewed and validated on its own before proceeding.

## Goals

- Reduce duplicated experiment-runner code across `scripts/`.
- Move reusable logic into the `navigation_planta` package.
- Separate experiment configuration from execution mechanics.
- Reduce the amount of domain-model logic living in CLI scripts.
- Make result records, CSV generation, and plotting less fragile.
- Add enough tests to support refactoring safely.

## Non-goals

- Changing experiment semantics or published scenario definitions.
- Rewriting the whole repo in one pass.
- Replacing the external planning toolchain.
- Optimizing runtime unless the refactor naturally simplifies a hotspot.

## Guiding Constraints

- Preserve current experiment outputs and CLI behavior unless a phase explicitly changes them.
- Keep the implementation repositories as ground truth for behavior.
- Prefer small, reviewable changes over large restructures.
- Add tests before or during behavior-preserving refactors where practical.

## Phase 0: Baseline And Safety Net

Purpose: establish a stable baseline before structural changes.

### Scope

- Inventory current experiment entrypoints and their expected outputs.
- Identify a small set of representative commands to validate after each phase.
- Add minimal tests around the most fragile shared behaviors.

### Tasks

1. Define a validation matrix covering at least:
   - `run_grid_map_scenario.py`
   - `run_camara_scenario.py`
   - `run_camara_scenario_discretized.py`
   - `run_fd_scale_scenario.py`
   - `run_corridor_type_scale_scenario.py`
   - `run_mission_action_scale_scenario.py`
   - `run_combined_scenario.py`
   - `run_camara_prism_scenario.py`
2. Document, for each script:
   - main CLI arguments
   - expected output folder shape
   - CSV columns
   - plot filenames
3. Add a `tests/` directory if one does not exist.
4. Add focused tests for current stable utilities, especially:
   - `count_plan_actions()`
   - `plot_mode_summary()` with synthetic records
   - `plot_memory_summary()` with synthetic records
   - `write_problem_fast()` using a small generated problem
5. Add one smoke test for `MapGenerator.generate_domain_problem_files()`.

### Deliverables

- Baseline documentation for runner outputs.
- Initial automated test skeleton.
- A short checklist to run after each later phase.

### Suggested Tests

- Unit tests:
  - plan-file parsing with solved and unsolved plan files
  - plot generation creates expected image files
  - problem writer emits key predicates and typed objects correctly
- Smoke tests:
  - create a tiny graph, generate a problem, and confirm the output files exist

### Exit Criteria

- Basic tests run locally.
- The team has a repeatable way to verify that a refactor did not break outputs.

## Phase 1: Normalize Shared Planner Execution And Result Records

Purpose: stop duplicating the planner invocation and tuple schema handling.

### Scope

- Centralize planner execution and metric collection.
- Replace ad hoc positional tuples with typed records or structured dictionaries.

### Tasks

1. Introduce a shared result model in `navigation_planta`, for example:
   - `RunRecord`
   - `CategoricalRunRecord`
   - or one flexible dataclass with optional fields
2. Define standard fields such as:
   - `mode`
   - `x_value`
   - `x_label`
   - `owltopddl_time`
   - `planning_time`
   - `action_count`
   - `peak_memory`
   - `run_id`
3. Migrate `run_grid_map_scenario.py`, `run_fd_scale_scenario.py`, `run_corridor_type_scale_scenario.py`, `run_mission_action_scale_scenario.py`, `run_combined_scenario.py`, and Cámara scripts to build these records instead of raw tuples.
4. Move all Fast Downward timing, plan counting, and peak-memory measurement to one shared helper API.
5. Add CSV serialization helpers that accept structured records instead of each script maintaining its own `np.savetxt` schema manually.

### Deliverables

- Shared record type(s).
- Shared CSV writer for experiment results.
- Shared planner execution helper used consistently by all PDDL experiment scripts.

### Suggested Tests

- Unit tests for record-to-CSV conversion.
- Unit tests for backward-compatible CSV column ordering where needed.
- Unit tests for planner metric extraction with mocked subprocess output or synthetic plan files.

### Exit Criteria

- PDDL runner scripts no longer depend on positional tuple unpacking for core metrics.
- CSV writing logic is reused rather than duplicated.

## Phase 2: Extract A Shared Experiment Runner Framework

Purpose: remove duplicated orchestration across experiment entrypoints.

### Scope

- Introduce a package-level experiment framework for:
  - run folder creation
  - mode handling
  - adaptive vs no-adaptation preprocessing
  - repeated trials
  - results saving
  - plot dispatch

### Tasks

1. Create a new module such as:
   - `navigation_planta.experiment_runner`
   - or `navigation_planta.experiments.core`
2. Define a small abstraction for an experiment, for example:
   - `ExperimentSpec`
   - `SweepSpec`
   - `ScenarioFactory`
3. Centralize the repeated lifecycle:
   - build generator
   - generate graph or load map
   - emit problem
   - run OWLToPDDL if adaptive
   - run planner
   - return record
4. Add a reusable loop helper for numeric sweeps and categorical sweeps.
5. Convert the standalone scripts into thin entrypoints that primarily:
   - define constants
   - parse args
   - instantiate specs
   - call shared execution
6. Remove recursive retry patterns and replace them with bounded retries and explicit failure reporting.

### Deliverables

- Shared experiment framework in the package.
- Thinner entrypoint scripts with less orchestration logic.
- Bounded retry behavior.

### Suggested Tests

- Unit tests for the shared run loop using fake scenario factories.
- Tests for retry behavior:
  - retries on transient `OWLToPDDL` failure
  - stops after configured limit
- Tests that output directories and filenames are generated consistently.

### Exit Criteria

- The experiment scripts are mostly configuration plus argument parsing.
- Adding a new sweep no longer requires copy-pasting a whole existing script.

## Phase 3: Move Scenario Variants Out Of Scripts And Into The Package

Purpose: separate domain logic from CLI scripts.

### Scope

- Move script-local generator subclasses into `navigation_planta/`.
- Consolidate adaptive and no-adaptation variants around shared feature composition.

### Tasks

1. Move these script-local classes into package modules:
   - `CorridorTypeMapGenerator`
   - `MissionActionMapGenerator`
   - `CombinedScenarioMapGenerator`
2. Create a dedicated module, for example:
   - `navigation_planta.scenario_variants`
   - or `navigation_planta.experiments.scenarios`
3. Keep scripts responsible only for choosing which variant to use.
4. Review `NoAdaptationMapGenerator` and `MissionActionNoAdaptationMapGenerator` so their logic lives beside the adaptive variants they correspond to.
5. Standardize naming of scenario variants and their parameters:
   - corridor-type scaling
   - mission-action scaling
   - combined stress scenario

### Deliverables

- Package-level scenario classes.
- Cleaner separation between scenario modeling and script entrypoints.

### Suggested Tests

- Unit tests for each scenario variant:
  - generated fluents exist
  - generated goals exist
  - expected extra predicates are present in the output problem
- Regression tests comparing selected generated PDDL fragments before and after the move.

### Exit Criteria

- No experiment-specific `MapGenerator` subclasses remain defined in CLI scripts.
- Scenario behavior is testable without going through a script.

## Phase 4: Replace Subclass Duplication With Reusable Feature Composition

Purpose: remove repeated `generate_domain()` and `generate_problem()` logic.

### Scope

- Refactor scenario-specific extensions into reusable composable pieces.

### Tasks

1. Identify the repeated feature families:
   - extra corridor requirement types
   - extra mission-action waypoint and completion goals
   - combined scenario features
2. Introduce reusable helpers or mixins such as:
   - corridor-feature builders
   - mission-goal builders
   - graph annotation helpers
3. Refactor `generate_domain()` and `generate_problem()` so the base `MapGenerator` delegates to extension hooks rather than forcing whole-method overrides.
4. Reduce duplication between:
   - `MissionActionMapGenerator`
   - `MissionActionNoAdaptationMapGenerator`
   - `CorridorTypeMapGenerator`
   - `CombinedScenarioMapGenerator`
5. Keep the final public scenario classes simple wrappers around composed features.

### Deliverables

- Feature-oriented extension layer for map and PDDL generation.
- Smaller scenario classes with less override-heavy logic.

### Suggested Tests

- Unit tests for feature helpers in isolation.
- Regression tests ensuring generated adaptive and no-adaptation problems still contain the expected scenario-specific elements.

### Exit Criteria

- Scenario variants reuse feature-building blocks instead of duplicating large methods.
- Adding a new experiment dimension becomes additive rather than copy-paste-driven.

## Phase 5: Split `map_generator.py` By Responsibility

Purpose: reduce the size and conceptual load of the central module.

### Scope

- Break `map_generator.py` into focused modules while preserving behavior.

### Tasks

1. Separate graph creation and manipulation into a module such as:
   - `graph_generation.py`
2. Separate PDDL problem/domain construction and serialization into:
   - `pddl_builder.py`
   - `pddl_writer.py`
3. Separate map loading, discretization, and plotting into:
   - `map_io.py`
   - `plotting.py`
4. Decide whether `MapGenerator` remains a facade or is replaced by smaller collaborators.
5. Remove any example/demo code from module tails if it does not belong in production code.
6. Replace wildcard imports in the package with explicit imports where practical.

### Deliverables

- Smaller package modules with clearer boundaries.
- Reduced cognitive load in the former `map_generator.py`.

### Suggested Tests

- Existing tests should continue to pass unchanged.
- Add module-focused tests:
  - graph connectivity generation
  - discretization behavior
  - PDDL serialization outputs

### Exit Criteria

- No single module acts as generator, serializer, plotter, loader, and solver at once.
- The package layout communicates responsibilities clearly.

## Phase 6: Standardize Plotting And Reporting Utilities

Purpose: remove repeated plotting and output-format code.

### Scope

- Centralize common plotting/reporting behavior while preserving experiment-specific plots.

### Tasks

1. Introduce shared plotting APIs for:
   - numeric sweep plots with mean and standard deviation
   - categorical comparison plots
   - memory summary plots
2. Standardize figure style, labels, legend handling, and output naming rules.
3. Move the comparison plotting logic from `run_all_experiments.py` into a shared plotting/reporting module.
4. Ensure the Cámara-specific plotting rules remain supported as an explicit special case rather than ad hoc branching in multiple scripts.
5. Standardize summary text/report file generation where still needed.

### Deliverables

- Shared plotting/reporting module.
- Simpler experiment scripts and simpler `run_all_experiments.py`.

### Suggested Tests

- Plot helper tests using synthetic data.
- Tests for expected output filenames per plot kind.
- Tests for handling missing or empty action-count series.

### Exit Criteria

- Scripts do not each maintain their own bespoke plotting boilerplate.
- Reporting behavior is centralized and easier to evolve.

## Phase 7: Remove Legacy Runner

Purpose: eliminate the old execution path once the shared experiment framework is in place.

### Scope

- Remove `scripts/runner.py` and standardize on explicit experiment entrypoints.

### Tasks

1. Delete `scripts/runner.py`.
2. Remove references to it from documentation and repo guidance files.
3. Ensure experiment execution guidance points to the standalone scripts and `run_all_experiments.py`.

### Deliverables

- Legacy runner removed.
- Documentation updated to describe the canonical entrypoints.

### Suggested Tests

- No dedicated runner tests needed after removal.

### Exit Criteria

- The repo has one clear experiment-execution architecture, not two overlapping ones.

## Phase 8: Documentation Cleanup

Purpose: align docs with the refactored structure.

### Scope

- Update repo documentation once structural changes land.

### Tasks

1. Update `README.md` to describe:
   - the new package structure
   - canonical entrypoints
   - how to add a new experiment
2. Update any repo-local context docs if needed.
3. Add a short developer-oriented architecture note describing:
   - package modules
   - scenario extension model
   - experiment runner flow
4. If `run_all_experiments.py` changes significantly, document its current expected outputs.

### Deliverables

- Updated user and developer documentation.

### Suggested Tests

- No code tests required, but command examples should be spot-checked manually.

### Exit Criteria

- Repo docs reflect the actual architecture.

## Recommended Execution Order

Recommended order:

1. Phase 0
2. Phase 1
3. Phase 2
4. Phase 3
5. Phase 4
6. Phase 5
7. Phase 6
8. Phase 7
9. Phase 8

## Suggested Milestones

- Milestone A:
  - Phases 0 and 1 complete
  - shared records and baseline tests in place
- Milestone B:
  - Phases 2 and 3 complete
  - experiment scripts materially thinner
- Milestone C:
  - Phases 4 and 5 complete
  - scenario logic and package boundaries cleaned up
- Milestone D:
  - Phases 6 to 8 complete
  - plotting, docs, and legacy runner removal finalized

## Cross-Phase Risks To Watch

- Silent changes in CSV schema breaking downstream plotting.
- Behavior drift in generated PDDL while simplifying code structure.
- Plot changes that alter figure interpretation even if data is unchanged.
- Over-generalizing too early and making the framework harder to understand than the duplication it replaces.
- Accidentally breaking reproducibility paths documented in the README.

## Review Checklist For Each Phase

- Does this phase reduce duplication without changing experiment intent?
- Are CLI arguments and output folders still compatible, or explicitly documented if changed?
- Are generated CSV columns still correct?
- Are plots still created with the expected filenames?
- Are scenario-specific predicates and goals still present in generated PDDL?
- Are the new abstractions easier to understand than the code they replace?
- Were tests added or updated for the refactor surface touched in this phase?
