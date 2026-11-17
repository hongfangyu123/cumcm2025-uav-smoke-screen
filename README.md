# Coordinated smoke-screen deployment by a UAV swarm (CUMCM 2025, Problem A)

**China Undergraduate Mathematical Contest in Modeling (CUMCM) 2025 · Second Prize, Guangdong Province**
Team project, 72-hour contest · Python + MATLAB

Several drones each release smoke canisters. Each canister bursts into a cloud of 10 m radius that sinks slowly and lasts 20 seconds. The task was to choose every drone's heading and speed, and each canister's release and burst times, so that the clouds block the line of sight between incoming missiles and a protected cylindrical target for as long as possible. The difficulty grows from one drone with one canister to five drones and up to 15 canisters against three missiles.

The paper (in Chinese) is titled *基于改进遗传算法的无人机集群烟幕协同优化*, "Coordinated smoke-screen optimisation for UAV swarms based on an improved genetic algorithm".

---

## Approach

**1. Kinematics.** The missile flies in a straight line at 300 m/s. Each drone flies level at 70–140 m/s. A released canister keeps the drone's horizontal velocity and falls under gravity. After it bursts, the cloud centre sinks at 3 m/s.

**2. Occlusion test (geometry).** The target is a cylinder (radius 7 m, height 10 m), sampled at several hundred points on its surface. At each time step the target counts as hidden if every segment from the missile to a sample point passes through a smoke sphere. Each check is a segment–sphere intersection, solved with the discriminant of a quadratic.

**3. Objective.** Step through time and add up the total time the target stays hidden. Overlapping clouds count only once.

**4. Optimisation.** The objective is non-smooth and has many local optima. We used a genetic algorithm and improved it step by step as the problems grew:

| Problem | Setting | Decision variables | Method | Result (total occlusion time) |
|---|---|---|---|---|
| Q1 | 1 drone, 1 canister, fixed plan | none (evaluation only) | time-stepping simulation | 1.47 s |
| Q2 | 1 drone, 1 canister | 4 | standard GA | 4.60 s |
| Q3 | 1 drone, 3 canisters (≥ 1 s apart) | 8 | improved GA: elitist gene retention, rank-based selection, adaptive crossover/mutation rates, local search every 20 generations | 6.83 s |
| Q4 | 3 drones, 1 canister each | 12 | grouped encoding per drone, bounded search space, Latin hypercube initialisation, layered mutation | 11.2 s |
| Q5 | 5 drones × up to 3 canisters, 3 missiles | 60 | cell-array encoding, directional mutation that steers each drone towards the missiles' paths | 21.07 s (M1 7.63 s, M2 8.06 s, M3 3.35 s) |

## Code

| File | What it does |
|---|---|
| [`code/T1.py`](code/T1.py) | Q1: simulates the fixed plan and computes occlusion time; plots the occlusion status and distance over time |
| [`code/T2.m`](code/T2.m) | Q2: single drone and canister, optimised with MATLAB `ga` |
| [`code/T3.m`](code/T3.m) | Q3: hand-written improved GA for three canisters, exports `result1.xlsx` |
| [`code/T4.m`](code/T4.m) | Q4: three-drone coordination (MATLAB `ga` with grouped variables), exports `result2.xlsx` |
| [`code/T5.m`](code/T5.m) | Q5: five drones against three missiles, hand-written improved GA, exports `result3.xlsx` |

Requirements: Python 3 with NumPy and Matplotlib for `T1.py`; MATLAB R2019a or later; `T2.m` and `T4.m` also need the Global Optimization Toolbox (`ga`), while `T3.m` and `T5.m` use a hand-written GA.
