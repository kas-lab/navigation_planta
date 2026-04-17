# Suggested commands

## General Linux utilities
- `pwd`
- `ls`
- `find . -maxdepth 2 -type d | sort`
- `rg --files`
- `rg -n 'pattern' path/`
- `sed -n '1,200p' path/to/file`
- `git status`
- `git diff`

## Build / environment
- Build Docker image:
  - `docker build -t navigation_planta .`
- Open interactive container with repo mounted:
  - `docker run --rm -it --name navigation_planta -v $PWD/src/navigation_planta:/navigation_pddl_tomasys/ -v /etc/localtime:/etc/localtime:ro navigation_planta:latest bash`
- Manual Python dependency install:
  - `pip install psutil networkx numpy==1.26.4 matplotlib scipy unified-planning "unified-planning[fast-downward]"`

## Main entrypoints
- Run grid-map scalability experiment:
  - `python scripts/run_grid_map_scenario.py`
- Run Cámara PDDL scenario:
  - `python scripts/run_camara_scenario.py`
- Run Cámara PRISM scenario:
  - `python scripts/run_camara_prism_scenario.py`
- Run all experiments sequentially:
  - `python scripts/run_all_experiments.py`
  - `python scripts/run_all_experiments.py --runs 5`
  - `python scripts/run_all_experiments.py --skip camara_prism combined`

## Standalone map generation
- `python scripts/map_generator.py`

## Toolchain commands
- Example OWL to PDDL conversion:
  - `OWLToPDDL.sh --owl=owl/navigation_with_imports.owl --tBox --inDomain=pddl/domain_sas.pddl --outDomain=pddl/domain_sas_created.pddl --aBox --inProblem=problem.pddl --outProblem=problem_created.pddl --replace-output --add-num-comparisons`
- Example Fast Downward run:
  - `fast-downward.py pddl/domain_sas_created.pddl pddl/problem_created.pddl --search "astar(blind())"`
- Example PRISM run:
  - `prism navigate_map_one_path.prism -pf 'R{"time"}min=? [ F stop ]' -exportstrat stdout -const INITIAL_BATTERY=5000,INITIAL_LOCATION=1,TARGET_LOCATION=5,INITIAL_CONFIGURATION=1`

## Validation-oriented commands
- Build the Docker image to validate environment reproducibility:
  - `docker build -t navigation_planta .`
- Run a representative experiment script after planner/domain changes:
  - `python scripts/run_grid_map_scenario.py`
  - or `python scripts/run_camara_scenario.py`
- When isolating performance issues, measure the stages separately instead of running the whole suite.