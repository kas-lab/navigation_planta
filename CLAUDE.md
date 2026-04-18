# CLAUDE.md

> **Mirror policy:** The Codex-facing equivalent of this file is `AGENTS.md`. Keep both files in sync when updating commands, pipeline facts, or gotchas.

## Project Scope

Standalone PLANTA navigation scenario used in the PLANTA paper experiments:

- Grid-map scalability experiments (vary map size, measure planning time)
- Camara-style comparison scenario (fixed map from Cámara et al. 2020)
- OWL-to-PDDL and Fast Downward planning integration

No ROS required. All experiments run as plain Python scripts.

## Repository Layout

| Path | Role |
|---|---|
| `scripts/run_camara_scenario.py` | Camara map experiment entrypoint |
| `scripts/run_grid_map_scenario.py` | Grid scalability experiment entrypoint |
| `scripts/runner.py` | **Base class only** — not an entrypoint |
| `scripts/map_generator.py` | Map generation, plotting, and PDDL problem serialization |
| `scripts/prism_model_generator.py` | PRISM model generation (Camara-style comparison) |
| `pddl/domain_sas.pddl` | Hand-written base PDDL domain — the only file to edit for domain changes |
| `pddl/domain.pddl` | Baseline domain without TOMASys (for reference only, not used in experiments) |
| `owl/navigation.owl` | TOMASys ontology — baseline (N=6 FDs) |
| `owl/experiment_fd_scalability/` | FD-scaling OWL series (N=2,4,6,8,10,12,15); **cumulative** — each file contains all FDs from smaller N |
| `map_camara_2020_paper/` | JSON map assets for the Camara comparison scenario |
| `results/` | Generated experiment outputs (gitignored) |

## Pipeline

Each experiment script runs these steps in order:

1. **Map → problem.pddl** — `map_generator.py` builds the waypoint graph and serializes it as a PDDL problem file.
2. **OWL + base domain → created files** — `OWLToPDDL.sh` takes `owl/navigation.owl` and `pddl/domain_sas.pddl` as inputs and produces `domain_created.pddl` + `problem_created.pddl`.
3. **Planning** — `fast-downward.py` runs greedy best-first search with the FF heuristic and preferred operators (`lazy_greedy([ff()], preferred=[ff()])`) on the created files.
4. **Results** — planning times saved to `results/planning_times.csv`.

Only `domain_sas.pddl` and `navigation.owl` are hand-authored. Everything else is generated.

## Common Commands

Install dependencies:
```bash
pip install networkx "numpy==1.26.4" matplotlib scipy unified-planning "unified-planning[fast-downward]"
```

Run Camara scenario (from `src/navigation_planta/`):
```bash
python scripts/run_camara_scenario.py
```

Run grid scalability experiment:
```bash
python scripts/run_grid_map_scenario.py
```

Run via Docker:
```bash
docker build -t navigation_planta .
docker run --rm -it --name navigation_planta \
  -v $PWD/src/navigation_planta:/navigation_pddl_tomasys/ \
  -v /etc/localtime:/etc/localtime:ro \
  navigation_planta:latest bash
```

## OWL File Series (FD Scaling)

The `owl/experiment_fd_scalability/navigation_fd_*.owl` files are **cumulative**: fd_10 contains everything in fd_8 plus new FDs.
When adding or changing a component/FD that first appears at level N, update **all files from N onward**.

| File | N | New components introduced |
|---|---|---|
| `navigation_fd_2.owl` | 2 | — |
| `navigation_fd_4.owl` | 4 | — |
| `navigation_fd_6.owl` | 6 | — |
| `navigation_fd_8.owl` | 8 | `c_wheel_encoder` |
| `navigation_fd_10.owl` | 10 | `c_sick_lidar` |
| `navigation_fd_12.owl` | 12 | `c_imu` |
| `navigation_fd_15.owl` | 15 | `c_radar` |

`fd_sick_lidar` requires `c_sick_lidar` (not generic `c_lidar`). `fd_lidar_inertial` requires both `c_lidar` and `c_imu`. `requiresC` is multi-valued — comma-separate multiple components.

## Timing Rule

The reported **planning time** covers only the `fast-downward.py` call. Map generation, problem-file writing, and `OWLToPDDL.sh` happen before the timer starts. Do not equate wall-clock experiment time with the planning times in the CSV.

## Performance Notes

- `map_generator.py` generates connected grid maps by construction (not by retry). `generate_grid_graph()` keeps a connected subset of cells, adds 4-neighbor edges, and removes only edges outside a spanning tree.
- Problem files use `write_problem_fast()` instead of `unified_planning`'s generic `PDDLWriter.write_problem()`, which is too slow for large maps. If the problem gains metrics, timed effects, or HTN constructs, review this writer before reusing it.
- Keep `default_initial_value=False` for `robot_at` and `corridor` in the in-memory UP problem; the fast writer serializes only explicit initial values.

## Fast Downward Gotchas

- **Heuristics + axioms**: `ff()`, `add()`, `hmax()`, `cg()` all timeout (>60s on n=10) because axiom-layer relaxation is extremely expensive on this domain. Only `blind()` and `goalcount()` are feasible. Do not attempt to switch to relaxation-based heuristics.
- **CLI option order**: `--plan-file` is a *driver* option and must come **before** the domain/problem paths — `fast-downward.py --plan-file foo domain.pddl problem.pddl --search "..."`. Placing it after `--search` causes FD to exit non-zero immediately.
- **Plan file naming**: FD writes `{plan-file}.1`, `.2`, … (numbered), not the base name directly.
- **Subprocess cleanup**: `subprocess.run(timeout=N)` leaves the FD C++ child alive on timeout. Use `subprocess.Popen` + `start_new_session=True`, then `os.killpg(os.getpgid(proc.pid), signal.SIGKILL)` on `TimeoutExpired` to kill the whole process group.
- **Benchmark script**: `scripts/benchmark_search_strategies.py` tests all viable strategies on the combined scenario with configurable node sizes, runs, and timeout. Run with `--nodes 10,20,50 --runs 3 --timeout 60` to find the fastest strategy before updating experiment scripts.

## Validation Guidance

When debugging, measure stages in isolation:
1. Connected map generation
2. Problem-file generation (`write_problem_fast`)
3. `OWLToPDDL.sh`
4. `fast-downward.py`

When validating planner or domain changes, confirm both that `OWLToPDDL.sh` succeeds and that `fast-downward.py` finds a solution on a representative case.
