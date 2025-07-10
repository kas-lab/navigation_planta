# Navigation PLANTA

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/kas-lab/navigation_planta)](https://github.com/kas-lab/navigation_planta/releases)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.15855793.svg)](https://doi.org/10.5281/zenodo.15855793)


This repository applies PLANTA to a robot self-adaptation scenario based on the paper ["Software architecture and task plan co-adaptation for mobile service robots"](https://dl-acm-org.tudelft.idm.oclc.org/doi/abs/10.1145/3387939.3391591), and compares it with the solution proposed in that paper.

This repo contains the experimental setup used in the paper "Plan your Self-Adaptation! – Efficient Task and Architecture Co-adaptation Planning for Robots".

The remainder of this README explains how to reproduce the experiments.

## Docker

The first step is to get the docker image used for the experiments.

You can either build it locally:
```Bash
docker build -t navigation_planta .
```

Or you can download it with:
```Bash
docker pull ghcr.io/kas-lab/navigation_planta:main
```

Run with locally built image:
```bash
docker run --rm -it -v $HOME/navigation_planta_ws/src/navigation_planta/results:/navigation_pddl_tomasys/results -v /etc/localtime:/etc/localtime:ro navigation_planta:latest python scripts/runner.py
```

Run with github image:
```bash
docker run --rm -it -v $HOME/navigation_planta_ws/src/navigation_planta/results:/navigation_pddl_tomasys/results -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/navigation_planta:main python scripts/runner.py
```

**Note:** If you want to change the directory where the results are going to be saved in the host machine, replace `HOME/navigation_planta_ws/src/navigation_planta/results` with the path of the directory where you want the results to be saved.

## Manual installation

Install deps:
```
pip install networkx
pip install numpy==1.26.4
pip install matplotlib
pip install scipy
```

Install [Unified Planning](https://unified-planning.readthedocs.io/en/latest/getting_started/installation.html):

```bash
pip install unified-planning
```
Install fast-downward unified planning engine:

```bash
pip install unified-planning[fast-downward]
```

Build:

```bash
colcon build --symlink-install --packages-skip plansys2_downward_planner
```

## Generate random maps and test with fast-downard

In the main folder
```bash
python scripts/map_generator.py
ros2 run downward_ros fast-downward.py --alias lama-first pddl/domain.pddl pddl/problem.pddl
```

## Convert OWL ontology to PDDL

Normal version:

```
export PATH=$HOME/navigation_pddl_tomasys_ws/src/owl_to_pddl:$PATH
```

```bash
OWLToPDDL.sh --owl=owl/navigation.owl --tBox --inDomain=pddl/domain_sas.pddl --outDomain=pddl/domain_sas_created.pddl --aBox --inProblem=pddl/problem.pddl --outProblem=pddl/problem_created.pddl --replace-output --add-num-comparisons -Xmxn16g
```

ROS version:
```bash
ros2 run owl_to_pddl owl_to_pddl.py --ros-args -p owl_file:=owl/navigation.owl -p in_domain_file:=pddl/domain_sas.pddl -p out_domain_file:=pddl/domain_sas_created.pddl -p in_problem_file:=pddl/problem.pddl -p out_problem_file:=pddl/problem_created.pddl
```

## Run fast-downward solver

```
export PATH=$HOME/navigation_pddl_tomasys_ws/src/downward:$PATH
```

```
fast-downward.py  pddl/domain_sas_created.pddl pddl/problem_created.pddl --search "astar(blind())"
```

```bash
ros2 run downward_ros fast-downward.py --alias lama-first pddl/domain_sas_created.pddl pddl/problem_created.pddl
```

## Example of results

Parameters:

```python
self.num_nodes = 30
self.nodes_skip = 0.1 # 10%
self.unconnected_amount = 0.15 # 10%
self.unsafe_amount = 0.25 # 10%
self.dark_amount = 0.25 # 10%
```

time:
```bash
Solution found.
Peak memory: 45500 KB
Remove intermediate file output.sas
search exit code: 0

INFO     Planner time: 0.15s

real	0m0,325s
user	0m0,270s
sys	0m0,054s
```

Parameters:
```python
self.num_nodes = 300
self.nodes_skip = 0.1 # 10%
self.unconnected_amount = 0.15 # 10%
self.unsafe_amount = 0.25 # 10%
self.dark_amount = 0.25 # 10%
```

time:
```bash
Solution found.
Peak memory: 50856 KB
Remove intermediate file output.sas
search exit code: 0

INFO     Planner time: 8.33s

real	0m8,517s
user	0m8,230s
sys	0m0,287s
```

## Run PRISM

```Bash
time prism navigate_map_one_path.prism -pf 'R{"time"}min=? [ F stop ]' -exportstrat stdout -const INITIAL_BATTERY=5000,INITIAL_LOCATION=1,TARGET_LOCATION=5,INITIAL_CONFIGURATION=1
```

```
time prism test.prism -pf 'R{"energy"}min=? [ F stop ]' -exportstrat stdout -const INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION=7,INITIAL_CONFIGURATION=1
```

```
time prism l52_l24.prism -javamaxmem 16g -cuddmaxmem 16g -pf 'R{"energy"}max=? [ F stop ]' -exportstrat stdout -const INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION=16,INITIAL_CONFIGURATION=1
```

```Bash
python scripts/run_camara_prism_scenario.py
```

Run with docker:
```bash
docker run --rm -it -v $HOME/navigation_pddl_tomasys_ws/src/navigation_pddl_tomasys/results:/navigation_pddl_tomasys/results pddl_tomasys:latest python scripts/run_camara_prism_scenario.py
```

```bash
docker run --rm -it -v $HOME/navigation_pddl_tomasys_ws/src/navigation_pddl_tomasys/results:/navigation_pddl_tomasys/results  ghcr.io/rezenders/navigation_pddl_tomasys:main python scripts/run_camara_prism_scenario.py
```

## Run grid map evaluation

```bash
docker run --rm -it -v $HOME/navigation_pddl_tomasys_ws/src/navigation_pddl_tomasys/results:/navigation_pddl_tomasys/results  ghcr.io/rezenders/navigation_pddl_tomasys:main python scripts/run_grid_map_scenario.py
```

Map example:

![map example](image.png)
