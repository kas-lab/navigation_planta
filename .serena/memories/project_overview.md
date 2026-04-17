# Navigation PLANTA overview

- Purpose: standalone PLANTA navigation scenario used in the PLANTA paper experiments.
- Main experiment families:
  - grid-map scalability experiments
  - Cámara-style comparison scenario on the fixed map assets in `map_camara_2020_paper/`
  - additional scaling scripts for function designs, corridor types, mission actions, and combined scalability
- Core pipeline:
  1. generate or load a map
  2. serialize a PDDL problem from Python
  3. run `OWLToPDDL.sh` to merge ontology/domain/problem inputs
  4. run `fast-downward.py` to solve the resulting planning problem
  5. write CSV/plot/result artifacts under `results/`
- Tech stack:
  - Python experiment scripts using `networkx`, `numpy`, `matplotlib`, `psutil`, and `unified-planning`
  - external tools: Java 17, PRISM, Fast Downward, OWLToPDDL (`dlToPlanning`)
  - Docker image for reproducible execution
  - ROS2 metadata/package skeleton exists (`package.xml`, `CMakeLists.txt`, `src/navigation_pddl_tomasys.cpp`), but the experiments are run as plain Python scripts and AGENTS/CLAUDE note that ROS is not required for the experiment workflows.
- Important repo facts:
  - `scripts/runner.py` is a base class module, not the main entrypoint to use directly for ordinary experiment runs.
  - The planning time reported by the grid experiment measures only the `fast-downward.py` call, not the full wall-clock runtime.
  - `map_generator.py` uses connected-map generation by construction and a custom `write_problem_fast()` serializer for performance.
  - Keep `default_initial_value=False` for `robot_at` and `corridor`; the fast PDDL writer depends on explicit initial values only.
- CI: the GitHub workflow only builds and pushes the Docker image; it does not run a separate lint/test suite.