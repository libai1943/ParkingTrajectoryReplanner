# ParkingTrajectoryReplanner

MATLAB demo for **online trajectory replanning during automated parking under sudden environmental changes**, based on the **parallel stitching** method proposed in:

> B. Li, Z. Yin, Y. Ouyang, Y. Zhang, X. Zhong, and S. Tang,  
> “Online trajectory replanning for sudden environmental changes during automated parking: A parallel stitching method,”  
> *IEEE Transactions on Intelligent Vehicles*, vol. 7, no. 3, pp. 748–757, 2022.  
> DOI: **10.1109/TIV.2022.3156429**

This repository provides the demonstration code used to illustrate the main idea of the paper: when an originally planned parking trajectory becomes invalid because of a newly appeared obstacle, the vehicle does not simply stop and restart the whole planning process. Instead, an **evasive trajectory** is generated and connected to the still-executable part of the original trajectory through a set of candidate **connective trajectories**. These candidates can be evaluated independently and therefore naturally support parallel computation.

The final replanned trajectory is constructed by selecting a feasible, low-cost stitching combination.

---

## Motivation

Trajectory replanning for automated parking is considerably different from local obstacle avoidance in ordinary on-road driving.

During parking, the vehicle may need to perform large steering actions, forward/reverse switching, and maneuvers in narrow spaces. When a previously unknown obstacle suddenly blocks the original trajectory, a small lateral modification may no longer be sufficient.

A replanner should therefore simultaneously address several issues:

- generate a substantially different evasive motion when necessary;
- respond quickly enough for online execution;
- preserve trajectory continuity;
- avoid unnecessary intermediate stops;
- respect vehicle kinematic and control limits; and
- maintain collision-free motion in constrained parking environments.

The parallel stitching framework was designed for this purpose.

---

## Core Idea: Parallel Stitching

Assume that the vehicle is following an originally planned parking trajectory when a new obstacle is detected.

The replanning procedure contains three major trajectory components:

```text
Original trajectory
        |
        | newly detected obstacle
        v
Evasive trajectory is generated
        |
        v
Candidate connective trajectories
        |
        v
Select a feasible low-cost stitching combination
        |
        v
Replanned trajectory
```

The key idea is that the newly generated evasive trajectory does not need to be adopted immediately from its first point.

Instead, the method searches for a suitable connection between:

1. a **future state on the original trajectory**, and
2. a **future state on the evasive trajectory**.

For different combinations of these two endpoints, independent connective-trajectory optimization problems can be solved.

Conceptually,

```text
Original trajectory
------o------o------o------o---------------->

          \      \      \
           \      \      \   connective candidates
            \      \      \
             o------o------o------o------------>
                      Evasive trajectory
```

If `Noriginal` candidate states are sampled from the original trajectory and `Nevasive` candidate states are sampled from the evasive trajectory, up to

```text
Noriginal × Nevasive
```

connection attempts can be considered.

Because these candidate problems are mutually independent, they are suitable for parallel computation. After feasibility and collision checks, the planner greedily selects a candidate that gives a satisfactory overall replanned trajectory.

This strategy avoids forcing the vehicle to immediately jump from its current state to a newly generated trajectory and also avoids unnecessary stop-and-replan behavior.

---

## Repository Workflow

The main entry is:

```matlab
RunMe
```

The execution sequence in `RunMe.m` is:

```matlab
LoadCase(case_id);
InitializeParams();
PlanOriginalTrajectory();

AssignSuddenObstacle();
PlanEvasiveTrajectory();
IdentifyConnectiveTrajectory();

CreateVideo();
```

Correspondingly, the full workflow is:

```text
Load parking scenario
        ↓
Initialize vehicle/planner parameters
        ↓
Load the original parking trajectory
        ↓
Introduce a sudden obstacle
        ↓
Generate an evasive trajectory
        ↓
Generate and evaluate connective trajectories
        ↓
Select the replanned trajectory
        ↓
Visualize the complete replanning process
```

---

## Quick Start

Clone the repository:

```bash
git clone https://github.com/libai1943/ParkingTrajectoryReplanner.git
cd ParkingTrajectoryReplanner
```

Open the folder in MATLAB and execute:

```matlab
RunMe
```

The default example is:

```matlab
case_id = 1;
```

Several other prerecorded test cases are provided:

```matlab
case_id = 14;
case_id = 20;
case_id = 36;
case_id = 39;
case_id = 96;
case_id = 100;
case_id = 108;
```

Simply modify the following line in `RunMe.m`:

```matlab
case_id = 1;
```

to select another example.

After the replanning computation is completed, the demo generates a visualization/video of the parking and replanning process in the current directory.

---

## Main Functions

### `RunMe.m`

Main entry of the demo.

It executes the complete online replanning pipeline:

```matlab
LoadCase
    ↓
InitializeParams
    ↓
PlanOriginalTrajectory
    ↓
AssignSuddenObstacle
    ↓
PlanEvasiveTrajectory
    ↓
IdentifyConnectiveTrajectory
    ↓
CreateVideo
```

For first-time users, this is the only script that needs to be executed.

---

### `LoadCase.m`

Loads a predefined parking problem.

```matlab
LoadCase(case_id)
```

Each `Case*.mat` file contains the parking task and obstacle environment, including:

- initial vehicle position;
- initial heading angle;
- target position;
- target heading angle; and
- polygonal obstacles.

These values are stored in the global structure:

```matlab
global params_
```

For example:

```matlab
params_.task.x0
params_.task.y0
params_.task.theta0

params_.task.xtf
params_.task.ytf
params_.task.thetatf
```

The obstacle polygons are stored in:

```matlab
params_.obstacle.obs
```

---

### `InitializeParams.m`

Defines the vehicle model, optimization parameters, Hybrid A* parameters, and online replanning settings.

Important vehicle parameters include:

```matlab
params_.vehicle.lw       % wheelbase
params_.vehicle.lf       % front overhang
params_.vehicle.lr       % rear overhang
params_.vehicle.lb       % vehicle width

params_.vehicle.vmax     % maximum velocity
params_.vehicle.amax     % maximum acceleration
params_.vehicle.phymax   % maximum steering angle
params_.vehicle.wmax     % maximum steering rate
```

The vehicle geometry in the default setting is:

```matlab
wheelbase       = 2.8 m
front overhang  = 0.96 m
rear overhang   = 0.929 m
vehicle width   = 1.942 m
```

The planner also defines limits on velocity, acceleration, steering angle, and steering rate.

---

## Parameters Worth Playing With

Several parameters in `InitializeParams.m` are particularly useful for understanding the behavior of the algorithm.

### Hybrid A* Search

```matlab
params_.user.hybrid_astar_max_iter = 500;
```

Controls the maximum search effort of the Hybrid A*-based planning stage.

The discretization is configured through:

```matlab
params_.hybrid_astar.resolution_dx
params_.hybrid_astar.resolution_dy
params_.hybrid_astar.resolution_dtheta
```

---

### Optimization Resolution

```matlab
params_.opti.nfe = 100;
```

defines the number of finite elements used in the trajectory optimization problem.

Increasing this value gives a finer discretization but normally increases computational cost.

---

### Candidate Stitching Numbers

The default numbers of candidate sampling points are:

```matlab
params_.reopti.Noriginal = 5;
params_.reopti.Nevasive  = 6;
```

These parameters control the candidate states sampled from the original and evasive trajectories.

Their combination determines the underlying set of possible stitching attempts.

For example, the default setting corresponds conceptually to up to

```text
5 × 6 = 30
```

endpoint combinations.

This is one of the most interesting parameters to modify when experimenting with the parallel stitching idea.

---

### Replanning Time Settings

```matlab
params_.reopti.unit_time_to_replan  = 1.0;
params_.reopti.unit_time_to_connect = 0.2;
```

These parameters describe the timing assumptions used by the demo during replanning and connection.

The code also defines:

```matlab
params_.reopti.T_consider
```

as their combination.

---

### Stitching Horizon

```matlab
params_.reopti.maximium_end_stitching_horizon_length = 8.0;
```

limits how far into the future candidate stitching endpoints are considered.

---

### Safety Buffer

```matlab
params_.reopti.s_buffer = 2.0;
```

defines a safety-related longitudinal buffer used during the replanning procedure.

---

## `PlanOriginalTrajectory.m`

```matlab
PlanOriginalTrajectory()
```

loads the prerecorded original parking trajectory corresponding to the selected test case.

For example, when

```matlab
case_id = 1;
```

the function loads:

```text
1.mat
```

and retrieves:

```matlab
traj_original
```

The original trajectory is stored as:

```matlab
params_.traj_original
```

and contains the trajectory state information needed by the subsequent online replanning procedure.

The repository therefore focuses on **online replanning**, rather than repeatedly solving the original offline parking-planning problem every time the demo is executed.

---

## `AssignSuddenObstacle.p`

This function introduces the environmental change that triggers replanning.

```matlab
AssignSuddenObstacle()
```

A new obstacle is inserted such that the previously planned parking trajectory is no longer directly executable.

This simulates the central problem considered in the paper:

> a valid parking trajectory has already been generated, but an unexpected obstacle suddenly appears during execution.

The function returns a status flag. `RunMe.m` terminates if a valid sudden-obstacle configuration cannot be generated.

---

## `PlanEvasiveTrajectory.p`

```matlab
PlanEvasiveTrajectory()
```

generates an alternative trajectory that avoids the newly introduced obstacle.

The evasive trajectory provides a new route toward the parking goal, but it cannot in general be used directly from its beginning because the real vehicle continues moving while replanning is being performed.

The resulting trajectory is stored in:

```matlab
params_.traj_evasive
```

The main state quantities include:

```matlab
x
y
theta
v
phy
tf
```

where `phy` denotes the steering angle and `tf` denotes terminal time.

If no feasible evasive trajectory can be found, the code reports that the vehicle should switch to a fail-safe behavior.

---

## `IdentifyConnectiveTrajectory.p`

This is the central component associated with the **parallel stitching** concept.

```matlab
IdentifyConnectiveTrajectory()
```

The function considers candidate future states along both:

```text
the original trajectory
```

and

```text
the evasive trajectory
```

and attempts to generate dynamically feasible connective trajectories between them.

Conceptually, each candidate solves:

```text
future state on original trajectory
                ↓
        connective trajectory
                ↓
future state on evasive trajectory
```

Multiple endpoint combinations can therefore be evaluated.

Valid candidate connections are checked and compared, after which the selected three-segment trajectory is assembled approximately as:

```text
remaining original trajectory segment
                +
        connective trajectory
                +
        evasive trajectory segment
```

The resulting trajectory is stored in:

```matlab
params_.traj_replanned
```

If no valid connective trajectory is found, the demo treats the replanning attempt as unsuccessful and indicates that a fail-safe strategy should be used.

---

## `MeasureTrajCost.m`

The demo uses:

```matlab
cost = MeasureTrajCost(traj)
```

to evaluate trajectory cost.

In the current implementation:

```matlab
cost = traj.tf;
```

so the terminal time is directly used as the trajectory cost.

This makes the candidate-selection mechanism particularly easy to understand and modify.

For example, researchers interested in other criteria could extend this function to include terms such as:

```text
travel time
+ steering effort
+ acceleration effort
+ path length
+ clearance
```

without changing the high-level stitching architecture.

---

## Optimization Models

The repository includes two AMPL optimization models.

### `NLP2.mod`

`NLP2.mod` contains a trajectory optimization problem involving:

```text
x       vehicle x position
y       vehicle y position
theta   heading angle
v       longitudinal velocity
a       acceleration
phy     steering angle
w       steering rate
```

It also introduces front and rear reference points:

```text
xf, yf
xr, yr
```

and constrains these points inside prescribed spatial tunnels.

The optimization accounts for vehicle kinematics, state/control limits, boundary conditions, and spatial feasibility.

---

### `NLP3.mod`

`NLP3.mod` formulates a boundary-value trajectory optimization problem between two complete vehicle states.

Its objective is:

```text
minimize terminal time
```

subject to the discrete vehicle dynamics and constraints on:

```text
velocity
acceleration
steering angle
steering rate
heading evolution
```

The boundary values include not only position and heading but also motion states such as velocity and steering quantities.

This formulation is particularly suitable for generating a smooth connection between two states already located on two existing trajectories.

---

## AMPL and Ipopt

The numerical optimization layer uses **AMPL** and **Ipopt**.

Relevant files include:

```text
ampl.exe
ipopt.exe
ipopt.opt
NLP2.mod
NLP3.mod
rr2.run
rr3.run
```

The `.run` files call Ipopt, solve the nonlinear programming problems, and write optimized trajectory variables back to text files for MATLAB to load.

For example, the optimized variables include:

```text
x
y
theta
v
a
phy
w
tf
```

The repository includes the executables and libraries used by the original demo.

### Platform Note

The archived implementation is primarily **Windows-oriented**.

For example, the AMPL scripts use commands such as:

```text
del
```

and the repository contains Windows executables and DLLs.

Running the demo on Windows with MATLAB is therefore the most straightforward option.

---

## Visualization

After a replanned trajectory is successfully found,

```matlab
CreateVideo()
```

visualizes the replanning process.

The generated animation illustrates:

- the parking environment;
- the original trajectory;
- the newly introduced obstacle;
- the evasive maneuver;
- the stitching process; and
- the final replanned vehicle motion.

Video generation may take additional time after the trajectory computation is completed.

---

## Repository Structure

```text
ParkingTrajectoryReplanner/
│
├── RunMe.m
│   Main entry of the complete demo
│
├── InitializeParams.m
│   Vehicle, planning, optimization, and replanning parameters
│
├── LoadCase.m
│   Loads a predefined parking scenario
│
├── PlanOriginalTrajectory.m
│   Loads the prerecorded original parking trajectory
│
├── AssignSuddenObstacle.p
│   Introduces the sudden environmental change
│
├── PlanEvasiveTrajectory.p
│   Plans an obstacle-avoiding evasive trajectory
│
├── IdentifyConnectiveTrajectory.p
│   Generates/evaluates candidate stitching trajectories
│
├── MeasureTrajCost.m
│   Evaluates the trajectory cost
│
├── CreateDilatedCostmap.p
│   Builds the dilated map used for collision-related planning
│
├── CreateVideo.p
│   Generates the visualization/video
│
├── Arrow.p
│   Plotting utility
│
├── NLP2.mod
├── NLP3.mod
│   Nonlinear trajectory optimization models
│
├── rr2.run
├── rr3.run
│   AMPL execution scripts
│
├── ipopt.opt
│   Ipopt configuration
│
├── ampl.exe
├── ipopt.exe
│   Optimization executables used by the original implementation
│
├── Case*.mat
│   Parking scenarios
│
└── *.mat
    Prerecorded original parking trajectories
```

---

## About the `.p` Files

Several core components are distributed as MATLAB protected `.p` files rather than `.m` source files:

```text
AssignSuddenObstacle.p
PlanEvasiveTrajectory.p
IdentifyConnectiveTrajectory.p
CreateDilatedCostmap.p
CreateVideo.p
```

This repository is intended as a **research demonstration of the published method**, rather than a release of every internal source file.

The protected functions can be called normally from MATLAB and are already integrated into `RunMe.m`.

There is no need to open or modify them in order to reproduce the provided demonstrations.

---

## A Simple Way to Explore the Demo

For a first experiment:

1. Run the default case:

```matlab
case_id = 1;
RunMe
```

2. Change to another scenario:

```matlab
case_id = 39;
```

3. Compare the resulting evasive and replanned trajectories.

4. Modify:

```matlab
params_.reopti.Noriginal
params_.reopti.Nevasive
```

to investigate how the number of candidate stitching combinations affects the procedure.

5. Modify:

```matlab
MeasureTrajCost.m
```

if you want to experiment with a different trajectory-selection objective.

These changes provide a relatively simple way to understand the parallel stitching mechanism without modifying the protected implementation.

---

## Citation

If this repository or the parallel stitching method is useful in your research, please cite:

> B. Li, Z. Yin, Y. Ouyang, Y. Zhang, X. Zhong, and S. Tang,  
> “Online trajectory replanning for sudden environmental changes during automated parking: A parallel stitching method,”  
> *IEEE Transactions on Intelligent Vehicles*, vol. 7, no. 3, pp. 748–757, 2022.  
> DOI: 10.1109/TIV.2022.3156429

### BibTeX

```bibtex
@article{li2022online,
  title={Online trajectory replanning for sudden environmental changes during automated parking: A parallel stitching method},
  author={Li, Bai and Yin, Zhuyan and Ouyang, Yakun and Zhang, Youmin and Zhong, Xiang and Tang, Shiqi},
  journal={IEEE Transactions on Intelligent Vehicles},
  volume={7},
  number={3},
  pages={748--757},
  year={2022},
  doi={10.1109/TIV.2022.3156429}
}
```

---

## Reference

**Bai Li, Zhuyan Yin, Yakun Ouyang, Youmin Zhang, Xiang Zhong, and Shiqi Tang**,  
“Online Trajectory Replanning for Sudden Environmental Changes During Automated Parking: A Parallel Stitching Method,”  
*IEEE Transactions on Intelligent Vehicles*, 7(3): 748–757, 2022.

---

Copyright © 2022 Bai Li.

The demo source identifies the software license as **GNU General Public License v3.0**.
