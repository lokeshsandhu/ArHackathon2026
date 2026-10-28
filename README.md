# Robot Warriors — Amazon Robotics Hackathon 2026

🏆 **Winning solution at the Amazon Robotics Hackathon 2026.**

We built a coordinated routing algorithm for warehouse robots that carry inventory pods from storage to pick stations. The challenge combines weighted shortest paths, shared aisles, limited station capacity, and time-sensitive deliveries.

Our implementation lives in [`ar_hackathon/api/routing.py`](ar_hackathon/api/routing.py). It uses only the Python standard library and runs inside the organizer-provided simulator.

## How it works

The engine asks each idle robot for its next adjacent node, or `None` to wait. Pickups and deliveries happen automatically. Our router coordinates decisions using the full floor state:

- **Cached graph distances:** precompute shortest travel times and reuse them while the graph is unchanged.
- **Coordinated task assignment:** match robots to pickup tasks globally for small problems using a memoized assignment solver, with greedy assignment for larger problems.
- **Delivery-aware batching:** evaluate short delivery sequences and opportunistic pickups when a loaded robot has spare capacity.
- **Traffic reservations:** search routes through time as well as space, accounting for aisle occupancy, inbound robots, and station capacity.
- **Priority repair:** try a few alternative robot orderings and compare estimated delivery reward.
- **Dock clearance and repositioning:** move idle blockers out of the way and distribute unassigned robots toward storage.

Planning uses bounded horizons and heuristics; it does not guarantee a globally optimal schedule. The final move is checked against current capacity constraints.

## Run it locally

Requires Python 3.7+ as declared by the starter package. A virtual environment is recommended.

```bash
git clone https://github.com/lokeshsandhu/ArHackathon2026.git
cd ArHackathon2026
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python scripts/run_game.py test_cases/level3/test_case_6.json
```

On Windows, create the environment with `py -m venv .venv` and activate it with `.venv\Scripts\activate`.

The simulator and visualization tools install their own dependencies, including NetworkX, Plotly, NumPy, Requests, and Kaleido. The routing algorithm itself has no third-party dependencies.

Compare against the supplied baseline:

```bash
python scripts/run_game.py test_cases/level3/test_case_6.json --driver basic
```

For an interactive walkthrough:

```bash
python scripts/run_game.py test_cases/level2/test_case_4.json --step-by-step
```

## Practice-case results

Measured locally using the supplied simulator and the six included cases. These are reproducible practice results, **not the competition's hidden-test scores or official leaderboard**.

| Level | Case | Robot Warriors score | Baseline score | Pods delivered by our router |
| --- | --- | ---: | ---: | ---: |
| 1 | `test_case_1` | 90.48 | 83.53 | 1 / 1 |
| 1 | `test_case_2` | 84.11 | 27.29 | 3 / 3 |
| 2 | `test_case_3` | 81.89 | 74.08 | 2 / 2 |
| 2 | `test_case_4` | 85.35 | 70.31 | 6 / 6 |
| 3 | `test_case_5` | 83.19 | 54.13 | 4 / 4 |
| 3 | `test_case_6` | 78.94 | 42.61 | 8 / 8 |

Our router delivers **24 / 24 pods**, compared with **16 / 24** for the baseline.

Run every practice case on macOS or Linux:

```bash
for case in test_cases/level*/*.json; do
  python scripts/run_game.py "$case"
done
```

Each delivered pod contributes `100 × exp(-delivery_duration / 50)` before normalization by the total pod count. Undelivered pods contribute zero, so both completion rate and delivery speed matter.

## Visualize a run

```bash
python scripts/visualize.py test_cases/level3/test_case_6.json --format html
```

Open `visualization_output/animation.html` in a browser and use the playback controls to inspect robot movement. Blue robots are empty; green robots carry pods. HTML output avoids the Chrome requirement for Kaleido image exports. If the animation jumps on the first playback, rewind it using the slider.

## Repository guide

| Path | Purpose |
| --- | --- |
| [`ar_hackathon/api/routing.py`](ar_hackathon/api/routing.py) | Our routing and coordination algorithm |
| `ar_hackathon/engine/` | Organizer-provided simulation engine |
| `ar_hackathon/models/` | Graph, robot, pod, and test-case models |
| `ar_hackathon/examples/basic_driver.py` | Supplied baseline driver |
| `ar_hackathon/visualizers/` | Floor rendering and animation |
| `scripts/` | Simulation and visualization commands |
| `test_cases/` | Six practice scenarios and their JSON schema |
| [`docs/challenge.md`](docs/challenge.md) | Original challenge instructions and mechanics |

## Team and credits

**Robot Warriors**. Personal contact details and the historical submission URL are omitted from the published configuration.

The simulator, models, baseline, test cases, and submission tooling originate from the [Amazon Robotics hackathon starter repository](https://github.com/salamazon/ArHackathon2026). Our competition implementation is in `routing.py`. The original challenge instructions are preserved in `docs/challenge.md` for reference; historical submission instructions and endpoints may no longer be active.
