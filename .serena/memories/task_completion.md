# Task completion guidance

- This repository does not expose a formal lint/format/test command set in project config.
- The minimum meaningful validation depends on the change:
  - for experiment-runner or planner-path changes, run a representative scenario script
  - for OWL/domain/planner integration changes, verify both that `OWLToPDDL.sh` succeeds and that `fast-downward.py` still finds a solution
  - for performance work, isolate and measure the suspected stage instead of using end-to-end planner CSV timings alone
- Useful validation entrypoints:
  - `python scripts/run_grid_map_scenario.py`
  - `python scripts/run_camara_scenario.py`
  - `python scripts/run_all_experiments.py --runs N --skip ...` for broader sweeps
- Environment validation can also be done with:
  - `docker build -t navigation_planta .`
- Avoid using `scripts/runner.py` alone as a validation entrypoint; it is a base class module, not the normal top-level script for experiment runs.