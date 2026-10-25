# Navigation PLANTA

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/kas-lab/navigation_planta)](https://github.com/kas-lab/navigation_planta/releases)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.15855793.svg)](https://doi.org/10.5281/zenodo.15855793)

Experimental setup for the paper **“Plan your Self-Adaptation! – Ontology-mediated Task and Architecture Co-adaptation Planning for Robots”**. This repository evaluates PLANTA through navigation scalability experiments and a comparison with the approach in [Cámara et al., SEAMS 2020](https://doi.org/10.1145/3387939.3391591).

Navigate the README:
[Installation](#installation) · [Run experiments](#run-experiments) · [Generate plots and tables](#generate-plots-and-tables) · [Manual tool examples](#manual-tool-examples) · [Tests](#tests) · [Adding a new experiment](#adding-a-new-experiment) · [Repository layout](#repository-layout)

**TL;DR:** Run the navigation full experimental suite with the upstream docker image saving the results in a `results` folder from where the command is executed:

```bash
docker run --rm \
  -v "$PWD/results":/navigation_planta/results -w /navigation_planta \
  ghcr.io/kas-lab/navigation_planta:main python scripts/run_all_experiments.py
```

## Repository layout

| Path | Contents |
|---|---|
| `scripts/` | Runnable experiment and analysis entrypoints |
| `scripts/helpers/` | Shared generators, experiment orchestration, and reporting helpers |
| `tests/` | Unit and integration tests |
| `owl/` | Ontologies for the adaptive scenarios |
| `pddl/` | Base and scenario-specific PDDL inputs |
| `prism/` | Static PRISM example models |
| `data/` | Cámara navigation map and associated assets |
| `requirements.txt` | Python runtime and test dependencies |
| `Dockerfile` | Complete experiment and rendering environment |

## Installation

### Docker — recommended

Build the image from the repository root:

```bash
docker build -t navigation_planta .
```

Alternatively, download the published image and assign the same local name:

```bash
docker pull ghcr.io/kas-lab/navigation_planta:main
docker tag ghcr.io/kas-lab/navigation_planta:main navigation_planta:latest
```

The image includes Python dependencies, pytest, Fast Downward, PRISM, OWL-to-PDDL, and the LaTeX/Poppler tools used to render plots and tables.

### Manual installation — Debian/Ubuntu

#### Python and system dependencies

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip \
  git cmake build-essential openjdk-17-jdk wget unzip

# Run from the repository root.
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

**Note:** remember to activate the virtual env if you start a fresh terminal: `source .venv/bin/activate`

`requirements.txt` includes test dependencies and Unified Planning's Fast Downward plugin. However, the experiments also require the standalone Fast Downward executable installed below.

Choose a directory for the external tools and configure Java:

```bash
export PLANTA_TOOLS="$HOME/planta-tools"
mkdir -p "$PLANTA_TOOLS"
```

If you dont have a `JAVA_HOME` environment variable setup, you need to set it. To check whether it is already set run: `env | grep JAVA_HOME`, if returns empty it means `JAVA_HOME` is not set. Make sure to have at least java 17.

Set `JAVA_HOME`:

```bash
export JAVA_HOME="$(dirname "$(dirname "$(readlink -f "$(command -v javac)")")")"
```

After setting it, save the current value in `~/.bashrc` for future Bash sessions:

```bash
echo "export JAVA_HOME=\"$JAVA_HOME\"" >> ~/.bashrc
```

#### PRISM

Download and Build PRISM 4.10.1:

```bash
git clone --branch v4.10.1 https://github.com/prismmodelchecker/prism.git \
  "$PLANTA_TOOLS/prism"
(cd "$PLANTA_TOOLS/prism/prism" && make && make test)
export PATH="$PLANTA_TOOLS/prism/prism/bin:$PATH"
# Make PRISM available in future Bash sessions.
echo "export PATH=\"$PLANTA_TOOLS/prism/prism/bin:\$PATH\"" >> ~/.bashrc
```

#### Fast Downward

Download and build Fast Downward release 24.06.1:

```bash
git clone --branch release-24.06.1 https://github.com/aibasel/downward.git \
  "$PLANTA_TOOLS/downward"
(cd "$PLANTA_TOOLS/downward" && ./build.py)
export PATH="$PLANTA_TOOLS/downward:$PATH"
# Make Fast Downward available in future Bash sessions.
echo "export PATH=\"$PLANTA_TOOLS/downward:\$PATH\"" >> ~/.bashrc
```

#### OWL-to-PDDL

Build dlToPlanning using its Gradle wrapper, which downloads the required Gradle distribution:

```bash
git clone https://github.com/remaro-network/dlToPlanning.git \
  "$PLANTA_TOOLS/dlToPlanning"
(cd "$PLANTA_TOOLS/dlToPlanning" && ./gradlew shadowJar)
export PATH="$PLANTA_TOOLS/dlToPlanning:$PATH"
# Make OWL-to-PDDL available in future Bash sessions.
echo "export PATH=\"$PLANTA_TOOLS/dlToPlanning:\$PATH\"" >> ~/.bashrc
```

#### LaTeX and image previews

PDF/PNG previews of the publication plots and tables require TeX Live and Poppler:

```bash
sudo apt-get install -y texlive-latex-extra texlive-pictures \
  texlive-fonts-recommended texlive-fonts-extra poppler-utils
```

These packages provide `pdflatex`, `kpsewhich`, `pdftoppm`, and the required LaTeX packages, including Libertine. They are not needed when exporting TeX sources with `--no-previews`.

## Run experiments

Run all commands from the repository root. With manual installation, activate `.venv` first.

### Docker execution environment

If you want to iteractively run experiments inside the docker container, run the container and open a shell with the checkout mounted:

```bash
docker run --rm -it \
  -v "$PWD":/navigation_planta -w /navigation_planta \
  navigation_planta:latest bash
```

Run the same Python commands below inside this shell. Source changes are immediately available, and outputs remain in the host checkout's `results/` directory. The repository itself is not installed.

### Full experiment suite

To run all navigation experiments from the paper, run:

```bash
python scripts/run_all_experiments.py
```

Or run the full suite directly with one Docker command from the repository root. These commands use the code included in the image and save results to the host's `results/` directory.

Using the published upstream image:

```bash
docker run --rm \
  -v "$PWD/results":/navigation_planta/results -w /navigation_planta \
  ghcr.io/kas-lab/navigation_planta:main python scripts/run_all_experiments.py
```

Using the locally built image:

```bash
docker run --rm \
  -v "$PWD/results":/navigation_planta/results -w /navigation_planta \
  navigation_planta:latest python scripts/run_all_experiments.py
```

These run all experiments with the default paper settings. Generate the publication plots and tables afterward using [Generate plots and tables](#generate-plots-and-tables).

**Note:** Running the full experiment suite takes approximately 44 hours... So you might consider running the experiment suite with a dettached `tmux` or `screen` session.  

The default suite runs the five scalability scenarios, discretized Cámara PDDL, and the Cámara PRISM baseline. It uses **10 repetitions**, with adaptive PDDL runs under **A* with the blind heuristic** and **eager greedy with the goal-count heuristic**. It also runs no-adaptation baselines for grid, discretized Cámara, mission-action, and combined scenarios.

Outputs are saved under `results/all_experiments/<timestamp>/`, with strategy-specific subdirectories, `camara_prism/`, and `comparison/`. Check the final status summary for failed experiments before using the results.

To customize a run, for example:

```bash
python scripts/run_all_experiments.py --runs 1 \
  --search 'astar(blind())' --skip camara_prism camara_discretized \
  camara_discretized_no_sas
```

`--runs` changes repetitions, not the scenario sizes. `--skip` selects individual run names. Adaptive and no-adaptation runs have separate names such as `grid_map` and `grid_map_no_sas`. Reduced or skipped runs are **not sufficient for the publication exporter**.

### Individual experiments

| Experiment | Name in the paper | Script in `scripts/` |
|---|---|---|
| Grid-map scalability | S1 | `run_grid_map_scenario.py` |
| FunctionDesign scalability | S2 | `run_fd_scale_scenario.py` |
| Corridor-type scalability | S3 | `run_corridor_type_scale_scenario.py` |
| Mission-action scalability | S4 | `run_mission_action_scale_scenario.py` |
| Combined scalability | S5 | `run_combined_scenario.py` |
| Discretized Cámara PDDL | PLANTA comparison (A* / eager greedy) | `run_camara_scenario_discretized.py` |
| Cámara PRISM baseline | Cámara et al. (replicated) | `run_camara_prism_scenario.py` |

For example, run the grid experiment or compare its adaptive and no-adaptation modes:

```bash
python scripts/run_grid_map_scenario.py
python scripts/run_grid_map_scenario.py --mode both --runs 1
```

PDDL entrypoints support `--mode adaptive|no-adaptation|both`. Individual scripts save results in their own timestamped directories under `results/`. Use each script's help for its options and defaults:

```bash
python scripts/run_grid_map_scenario.py --help
```

To run a single discretized Cámara experiment from waypoint 12 to waypoint 11, use the command below. Each range includes its first number but excludes its second.

```bash
python scripts/run_camara_scenario_discretized.py \
  --init 12 13 --goal 11 12 --runs 1
```

To benchmark search strategies on the combined scenario:

```bash
python scripts/benchmark_search_strategies.py --nodes 10,20,50 --runs 3 --timeout 60
```

**Interpreting measurements:** PDDL planning times cover the Fast Downward call, not map generation or OWL preprocessing. Memory is estimated peak aggregate RSS in MiB. The PRISM comparison optimizes remaining battery. PLANTA and PRISM retain different safety formulations, not identical movement feasibility or equivalent safety guarantees.

## Generate plots and tables

Experiment scripts generate runtime plots automatically. For publication data, LaTeX plots, and tables, post-process a **completed full-suite result directory**:

```bash
python scripts/generate_paper_artifacts.py "results/all_experiments/<timestamp>" \
  --output-dir results/paper_artifacts
```

This does not rerun experiments or edit manuscript sources. The publication profile requires the full parameter grids and 10 repetitions, both default adaptive strategies, PRISM, and A* no-adaptation results for grid, mission-action, and combined scalability. Missing, invalid, or incomplete inputs cause an error rather than being silently discarded.

The output bundle contains:

| Output | Contents |
|---|---|
| `data/` | Raw and processed CSVs, plus boxplot statistics |
| `plots/` | Includable PGFPlots sources and PDF/PNG previews |
| `tables/` | CSV/LaTeX summary tables and PDF/PNG previews |
| `summaries/` | Numerical summaries in CSV/JSON |
| `standalone/` | LaTeX wrappers for preview rendering |
| `manifest.json` | Input/output hashes, methods, and provenance |

| Option | Purpose |
|---|---|
| `--no-previews` | Export sources and data without PDF/PNG rendering or LaTeX dependency checks |
| `--dry-run` | Validate inputs and report the planned export without writing files |
| `--overwrite` | Replace a complete, unmodified bundle previously created by this exporter |
| `--latex-prefix PATH` | Set the bundle's relative path inside the destination LaTeX project |

For example, generate sources directly into a LaTeX project's bundle directory:

```bash
python scripts/generate_paper_artifacts.py "results/all_experiments/<timestamp>" \
  --output-dir /path/to/latex-project/navigation_experiment \
  --latex-prefix navigation_experiment --no-previews
```

The generated bundle README explains how to include the sources. Keep the export separate from the input results. Paper-artifact statistics use **sample standard deviation**, not standard error or confidence intervals; strategies are never pooled.

To rebuild only the experiment comparison plots from existing CSVs:

```bash
python scripts/regenerate_comparison_plots.py "results/all_experiments/<timestamp>"
```

This writes to the suite's `comparison/` directory and does not generate the publication bundle.

## Manual tool examples

The experiment scripts automate these steps. The examples below are for inspecting individual stages, using `results/manual/` rather than overwriting tracked inputs. Run them from the repository root, either inside the Docker shell or with the manual environment configured.

### Generate PDDL, convert OWL, and run Fast Downward

Create a small navigation problem:

```bash
mkdir -p results/manual
PYTHONPATH=scripts python - <<'PY'
import random
from helpers.map_generator import MapGenerator

random.seed(42)
generator = MapGenerator(num_nodes=9, nodes_skip=0)
generator.generate_connected_grid_map()
generator.generate_domain_problem_files(problem_filename="results/manual/problem.pddl")
PY
```

Combine it with the ontology and adaptive base domain, then plan:

```bash
OWLToPDDL.sh --owl=owl/navigation.owl --tBox \
  --inDomain=pddl/domain_sas.pddl --outDomain=results/manual/domain_created.pddl \
  --aBox --inProblem=results/manual/problem.pddl \
  --outProblem=results/manual/problem_created.pddl \
  --replace-output --add-num-comparisons

fast-downward.py --plan-file results/manual/plan \
  results/manual/domain_created.pddl results/manual/problem_created.pddl \
  --search 'astar(blind())'
```

### Generate and solve a PRISM model

Generate a model for one route in the Cámara map. `TARGET` is the final **path index**, not the original waypoint ID:

```bash
mkdir -p results/manual
TARGET=$(PYTHONPATH=scripts python - <<'PY'
from helpers.prism_model_generator import PrismModelgenerator

generator = PrismModelgenerator()
generator.load_json("data/map_camara_2020_paper/map-p2cp3.json")
route = generator.find_shortest_path(from_node=12, to_node=11)
generator.generate_prism_model("results/manual/route.prism", route)
print(len(route) - 1)
PY
)
prism results/manual/route.prism -pf 'R{"energy"}max=? [ F stop ]' \
  -exportstrat results/manual/strategy.txt:type=actions \
  -const "INITIAL_BATTERY=32560,INITIAL_LOCATION=0,TARGET_LOCATION=$TARGET,INITIAL_CONFIGURATION=1"
```

This uses the same remaining-battery objective as the PRISM experiment runner.

## Tests

Run all tests in the Docker shell or a fully configured manual environment:

```bash
python -m pytest -q tests
```

Use `-v` instead of `-q` for individual test names. Planner integration and rendering checks require the external tools above; a partial manual installation may skip tests or fail dependency checks.

## Adding a new experiment

1. Add or extend a generator in `scripts/helpers/scenario_variants.py` when new scenario behavior is needed.
2. Add a `scripts/run_<experiment>_scenario.py` entrypoint with its parameters and CLI options; reuse `scripts/helpers/experiment_runner.py` and `scripts/helpers/reporting.py` for PDDL sweeps and reporting.
3. Add focused tests under `tests/` for the generated problem and expected results.
4. If it belongs in the full suite, register it in `scripts/run_all_experiments.py` and the comparison configuration in `scripts/helpers/reporting.py`. Extend the publication exporter explicitly if the new experiment should appear in its output.
