# Dynamic Vehicle Routing with Stochastic Requests

## Description
The Dynamic Vehicle Routing Problem with Stochastic Requests (DVRPSR) is a prototypical problem in transportation logistics. The DVRPSR calls for an initial route plan and an online scheduling policy to route dynamically and in real-time a fleet of vehicles, in such a way that the number of customer requests accepted and served is maximized, and all vehicles return to the depot before a given deadline.

This repository contains the source code and datasets to allow the replication of the results from the paper:

[1] **Zhang, J., Luo, K., Florio, A.M., & Van Woensel, T. (2022). Solving Large-Scale Dynamic Vehicle Routing Problems with Stochastic Requests.**

A presentation summarizing the methodology proposed is available [here](DVRPSR-Presentation.pdf).

For a glimpse of the DVRPSR and some of the scheduling policies implemented, watch our project's visual abstract at [https://youtu.be/D57xNfU73as](https://youtu.be/D57xNfU73as).

![DVRPSR Snapshot](snapshot.png?raw=true|width=100)

The code implements the following **scheduling policies** for the DVRPSR:
* **Greedy policy**: Accepts dynamic requests in a greedy fashion, and update planned routes either by cheapest insertion or by complete reoptimization.
* **PFA policy**: Scheduling policy based on policy function approximation.
* **Rollout policy**: Applies the rollout algorithm with either the Greedy or the PFA policy as the base policy.
* **Potential-based policy**: Accepts dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to-go, which is computed by solving multiple-knapsack models. **Currently the best-performing online policy for the DVRPSR.**
* **Simplified potential-based policy**: Similar to the potential-based policy, but the reward-to-go is approximated by single-knapsack models.

In addition, code for the following **offline planners** is also provided:
* **Myopic planner**: Offline route planner that solves approximately a duration-constrained VRP by a column generation-based heuristic.
* **Potential-based planner**: Offline route planner that evaluates the expected reward-to-go of planned routes by single-knapsack models. **Currently the best performing offline planner for the DVRPSR.**

## Dependencies
The implementation requires:
* The [boost](https://www.boost.org) C++ libraries, which are installed by default in most Linux environments.
* The [CPLEX Optimization Studio](https://www.ibm.com/ca-en/products/ilog-cplex-optimization-studio), which is free for academic use. CPLEX is used **(i)** for solving the multiple-knapsack models, **(ii)** within the column generation procedure for generating offline route plans, and **(iii)** within a branch-and-cut Traveling Salesman Problem (TSP) algorithm for reoptimizing planned routes.

## Building and Running
A `Makefile` is provided for reference only. This should be adapted to match the specific host requirements, including `CPLEX` header files and libraries.

The code compiles into a single executable `dvrpsr`. The app allows several command line options:

```
$ ./dvrpsr 
usage: ./dvrpsr <mode> [...]
where <mode> =
	 0   Export Network Instance to TeX/TikZ
	 1   Generate Random Static Requests
	 2   Generate Random Dynamic Requests
	 3   Create Myopic Offline Plan
	 4   Create (simple) Potential-based Offline Plan
	 5   Simulate: Greedy Policy (GP)
	 6   Simulate: GP, with Reopt
	 7   Simulate: Rollout on GP (H=10)
	 8   Simulate: Rollout on GP (H=25)
	 9   Simulate: Rollout on GP (H=50)
	10   Simulate: Rollout on GP (H=100)
	11   Simulate: Rollout on GP, with Reopt (H=10)
	12   Simulate: Rollout on GP, with Reopt (H=25)
	13   Simulate: Rollout on GP, with Reopt (H=50)
	14   Simulate: Rollout on GP, with Reopt (H=100)
	18   Create (multiple) Potential-based Offline Plan
	19   Simulate: Simplified Potential-based Policy (S-PbP, H=50)
	20   Simulate: S-PbP (H=100)
	21   Simulate: PbP (H=50)
	22   Simulate: PbP (H=100)
	23   Export Offline Plan to TeX/TikZ
	24   Simulate: PFA (CI)
	25   Simulate: PFA (Reopt)
	26   Simulate: Rollout on PFA (H=10)
	27   Simulate: Rollout on PFA (H=25)
	28   Simulate: Rollout on PFA (H=50)
	29   Simulate: Rollout on PFA (H=100)
	30   Simulate: Rollout on PFA, with Reopt (H=10)
	31   Simulate: Rollout on PFA, with Reopt (H=25)
	32   Simulate: Rollout on PFA, with Reopt (H=50)
	33   Simulate: Rollout on PFA, with Reopt (H=100)
```

For example, to simulate the potential-based policy (PbP) on the Vienna network with ~0.4 requests per minute, uniformly distributed and time-invariant (UTI) request distribution and 5 vehicles under the provided trajectory and (potential-based) offline plan:
```
$ ./dvrpsr 21 V-0.4-UTI-5-50 ../reqs/V-0.4-UTI.1.req ../offline_plans/V-0.4-UTI-5.pb.op 
Instance(): code: V-0.4-UTI-5-50
loading Vienna network ...
Lambda: 0.4
space-time distribution: uti (0)
number of vehicles: 5
length of the service period: 600 minutes
setting up node types based on spatial dist. and clusters ...
pre-processing shortest-paths ...
2000/16080 done
4000/16080 done
6000/16080 done
8000/16080 done
10000/16080 done
12000/16080 done
14000/16080 done
16000/16080 done
finished pre-processing shortest-paths
readRequests: read 42 static and 228 dynamic requests
loading offline plan from ../offline_plans/V-0.4-UTI-5.pb.op ...
offline plan loaded!
route 0: duration: 207.14 requests: 6
route 1: duration: 211.287 requests: 7
route 2: duration: 210.043 requests: 6
route 3: duration: 213.585 requests: 11
route 4: duration: 263.337 requests: 12
total duration (5 routes): 1105.39
request arrived: [1.79696,8567,11.8254]
decision: [1,1]  (time: 2.036 s)
accepted requests: 1
[...]
```
After around 4 minutes (depending on the hardware) the simulation finishes and the app outputs summary statistics:
```
[...]
Simulation results:
static requests: 42
dynamic requests accepted: 136
dynamic requests rejected: 92
dynamic requests acceptance ratio: 0.596491
total duration of static and accepted dynamic requests: 1734.37
(min % over total service duration: 0.578125)
avg decision time: 0.88232 s
max decision time: 2.072 s
```
Note: the simulation takes considerably longer in larger instances. In the largest instance of the dataset (V-1.5-UTI-20), policy PbP (H=50) takes up to 150 seconds **per request** (depending on hardware). Other policies (e.g., rollout policies with H=25 or higher) are not suitable for very large instances.

## Documentation
The implementation is modular and follows closely the methodology proposed in [1]. Below, we provide a brief description of the contents of each module:

---

### Duration-constrained VRP Modules and Offline Planners:
`DVRPData.h`: Stores information about an instance of the duration-constrained VRP, and provides helper functions.

`DVRPLabel.h`: Represents a label within the pricing algorithm of the column generation procedure, which is used to compute the exact linear bound of the duration-constrained VRP. Provides associated functions for extending labels, verifying dominance rules, etc.

`DVRPLinearSolution.h`: Represents a fractional solution to the duration-constrained VRP.

`DVRPLinearSolver.h`: Implements column generation (except the pricing algorithm) and interfaces with the linear programming solver (CPLEX).

`DVRPPricing.h`: Pricing algorithm (labeling algorithm) for identifying routes with negative reduced costs.

`DVRPRoute.h`: Represents a route of the duration-constrained VRP and provides associated functions.

`DVRPSolution.h`: Represents a solution to the duration-constrained VRP (i.e., a set of routes).

`DVRPSolver.h`: Petal heuristic to choose the best route combination among all routes created by column generation.

`MPotentialPlanner.h`: Potential-based offline planner based on the multiple-knapsack potential approximation. Currently, this algorithm takes very long to converge. The offline planner based on simple-knapsack approximations `PotentialPlanner.h` is preferred for generating potential-based plans.

`MyopicPlanner.h`: Myopic offline planner.

`OfflinePlan.h`: Represents an offline plan: set of planned routes to serve static (or scheduled) requests.

`OfflinePlanner.h`: Abstract class (or *interface*) to be implemented by offline planners.

`PotentialPlanner.h`: Potential-based offline planner based on the single-knapsack potential approximation. **Currently the best performing offline planner for the DVRPSR.**

---

### DVRPSR Simulator and Online Scheduling Policies:
`BasePolicy.h`: Abstract class (or *interface*) to be implemented by policies that are employed as *base policies* within the rollout scheduling policy.

`Decision.h`: Represents a decision and its \`accept\', \`assign\' and \`routing\' components.

`GreedyPolicy.h`: Greedy scheduling policy, which always accept requests if it is feasible to do so.

`OnlinePolicy.h`: Abstract class (or *interface*) to be implemented by scheduling policies.

`PFAPolicy.h`: Policy function approximation-based scheduling policy.

`PGreedyPolicy.h`: Simplified potential-based policy (S-PbP): potential approximation via simple knapsack models.

`PlannedRoute.h`: Represents a planned route.

`PolicySimulator.h`: DVRPSR simulator that can be used with arbitrary scheduling policies.

`Request.h`: Represents a customer request (arrival time, node and duration).

`RolloutPolicy.h`: Rollout scheduling policy that can be used with arbitrary base policies.

`SimResults.h`: Stores the results of the simulation.

`State.h`: Represents a state of the dynamic system, including all vehicle states and the current request.

`mPGreedyPolicy.h`: Potential-based policy (PbP): potential approximation by multiple-knapsack models. **Currently the best-performing online policy for the DVRPSR.**

---

### Reoptimization Models (Branch-and-Cut TSP Algorithm):
`MaxFlowSolution.h`: Stores a solution to the maximum-flow problem.

`MaxFlowSolver.h`: Augmenting-path algorithm for solving the single source, single destination maximum-flow problem.

`MinCutSolution.h`: Stores a solution to the global minimum-cut problem.

`MinCutSolver.h`: Algorithm for finding the global minimum cut in a flow network.

`RouteReoptimizer.h`: Reoptimizes the requests served along a planned route by solving an open TSP.

---

### Other Modules:
`Data.h`: Interface with the instance object that represents an instance of the DVRPSR.

`Dijkstra.h`: Shortest-path algorithms.

`Instance.h`: Stores instance information. Provides functions for generating sample paths following the instance's spatiotemporal distribution of requests. These functions are used by the implemented scheduling policies and can be used in the implementation of new lookahead policies.

`Link.h`: Represents a road segment of the street network.

`Path.h`: Represents a path on the street network.

`PathTree.h`: Represents a tree -- used mostly for storing shortest-path trees.

`Stopwatch.h`: Implements a simple stopwatch. Credits to [Kyle Kloepper](https://isocpp.org/wiki/faq/wg21#kyle-kloepper).

`TikZExporter.h`: Collection of functions to export a state of the dynamic system to LaTeX/TikZ graphics.

