# Dynamic Vehicle Routing with Stochastic Requests

## Description
The Dynamic Vehicle Routing Problem with Stochastic Requests (DVRPSR) is a prototypical problem in transportation logistics. The DVRPSR calls for an initial route plan and an online scheduling policy to route dynamically and in real-time a fleet of vehicles, in such a way that the number of customer requests accepted and served is maximized, and all vehicles return to the depot before a given deadline.

This repository contains the source code and datasets to allow the replication of the results from the paper:

[1] **Zhang, J., Luo, K., Florio, A.M., & Van Woensel, T. (2022). Solving Large-Scale Dynamic Vehicle Routing Problems with Stochastic Requests.**

For a glimpse of the DVRPSR and some of the scheduling policies implemented, watch our project's visual abstract: [https://youtu.be/D57xNfU73as](https://youtu.be/D57xNfU73as)

More specifically, the code implements the following **scheduling policies** for the DVRPSR:
* **Greedy policy**: Accepts dynamic requests in a greedy fashion, and update planned routes either by cheapest insertion or by complete reoptimization.
* **PFA policy**: Scheduling policy based on policy function approximation.
* **Rollout policy**: Applies the rollout algorithm with either the Greedy or the PFA policy as the base policy.
* **Potential-based policy**: Accepts dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to-go, which is computed by solving multiple-knapsack models.
* **Simplified potential-based policy**: Similar to the potential-based policy, but the reward-to-go is approximated by single-knapsack models.

In addition, code for the following **offline planners** is also provided:
* **Myopic planner**: Offline route planner that solves approximately a duration-constrained VRP by a column generation-based heuristic.
* **Potential-based planner**: Offline route planner that evaluates the expected reward-to-go of planned routes by single-knapsack models.

## Dependencies
The implementation requires:
* The [boost](https://www.boost.org) C++ libraries, which are installed by default in most Linux environments.
* The [CPLEX Optimization Studio](https://www.ibm.com/ca-en/products/ilog-cplex-optimization-studio), which is free for academic use. CPLEX is used **(i)** for solving the multiple-knapsack models, **(ii)** within the column generation procedure for generating offline route plans, and **(iii)** within a branch-and-cut Traveling Salesman Problem (TSP) algorithm for reoptimizing planned routes.

## Usage

## Documentation
The implementation is modular and follows closely the methodology proposed in [1]. Below, we provide a brief description of the contents of each module:

### Duration-constrained VRP Modules and Offline Planners
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

`PotentialPlanner.h`: Potential-based offline planner based on the single-knapsack potential approximation. **Currently the best performing offline planner.**

### DVRPSR Simulator and Online Scheduling Policies
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

`mPGreedyPolicy.h`: Potential-based policy (PbP): potential approximation by multiple-knapsack models. **Currently the best-performing online policy.**

### Reoptimization Models (Branch-and-Cut TSP Algorithm)
`MaxFlowSolution.h`: Stores a solution to the maximum-flow problem.

`MaxFlowSolver.h`: Augmenting-path algorithm for solving the single source, single destination maximum-flow problem.

`MinCutSolution.h`: Stores a solution to the global minimum-cut problem.

`MinCutSolver.h`: Algorithm for finding the global minimum cut in a flow network.

`RouteReoptimizer.h`: Reoptimizes the requests served along a planned route by solving an open TSP.

### Other Modules
> `Data.h`: Interface with the instance object that represents an instance of the DVRPSR.

> `Dijkstra.h`: Shortest-path algorithms.

> `Instance.h`: Stores instance information. Provides functions for generating sample paths following the instance's spatiotemporal distribution of requests. These functions are used by the implemented scheduling policies and can be used in the implementation of new lookahead policies.

> `Link.h`: Represents a road segment of the street network.

> `Path.h`: Represents a path on the street network.

> `PathTree.h`: Represents a tree -- used mostly for storing shortest-path trees.

> `Stopwatch.h`: Implements a simple stopwatch. Credits to [Kyle Kloepper](https://isocpp.org/wiki/faq/wg21#kyle-kloepper).

> `TikZExporter.h`: Collection of functions to export a state of the dynamic system to LaTeX/TikZ graphics.

