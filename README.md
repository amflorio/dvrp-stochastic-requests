# Dynamic Vehicle Routing with Stochastic Requests

## Description
The Dynamic Vehicle Routing Problem with Stochastic Requests (DVRPSR) is a prototypical problem in transportation logistics. The DVRPSR calls for an initial route plan and an online scheduling policy to route dynamically and in real-time a fleet of vehicles, in such a way that the number of customer requests accepted and served is maximized, and all vehicles return to the depot before a given deadline.

This repository contains the source code and datasets to allow the replication of the results from the paper:

[1] **Zhang, J., Luo, K., Florio, A.M., & Van Woensel, T. (2022). Solving Large-Scale Dynamic Vehicle Routing Problems with Stochastic Requests.**

For a glimpse of the DVRPSR and some of the scheduling policies implemented, watch our project's visual abstract: [https://youtu.be/D57xNfU73as](https://youtu.be/D57xNfU73as)

More specifically, the code implements the following **scheduling policies** for the DVRPSR:
* **Greedy policy**: accepts dynamic requests in a greedy fashion, and update planned routes either by cheapest insertion or by complete reoptimization.
* **PFA policy**: scheduling policy based on policy function approximation.accepts dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to-go, which is estimated by policy function approximation.
* **Rollout policy**: applies the rollout algorithm with either the Greedy or the PFA policy as the base policy.
* **Potential-based policy**: accepts dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to-go, which is computed by solving a multiple-knapsack model.
* **Simplified potential-based policy**: as the potential-based policy, but the reward-to-go is approximated by single-knapsack models.

In addition, code for the following **offline planners** is also provided:
* **Myopic planner**: an offline route planner that solves approximately a duration-constrained VRP by a column generation-based heuristic.
* **Potential-based planner**: an offline route planner that evaluates the expected reward-to-go of planned routes by a single-knapsack model.

## Dependencies
The implementation requires:
* The [boost](https://www.boost.org) C++ libraries, which are installed by default in most Linux environments.
* The [CPLEX Optimization Studio](https://www.ibm.com/ca-en/products/ilog-cplex-optimization-studio), which is free for academic use. CPLEX is used **(i)** for solving the multiple-knapsack models, **(ii)** within the column generation procedure for generating offline route plans, and **(iii)** within a Traveling Salesman Problem algorithm for reoptimizing planned routes.

## Usage

## Documentation
The implementation is modular and follows closely the methodology proposed in [1]. Below, we provide a brief description of the contents of each module:

### Duration-constrained VRP Modules (Offline Planners)

`DVRPData.h`: Stores information about an instance of the duration-constrained VRP, and provides helper functions.

`DVRPLabel.h`: Represents a label within the pricing algorithm within the column generation procedure, which is used to compute the exact linear bound of the duration-constrained VRP. Provides associated functions for extending labels, verifying dominance rules, etc.

`DVRPLinearSolution.h`: Represents a fractional solution to the duration-constrained VRP.

`DVRPLinearSolver.h`: Implements column generation (except the pricing algorithm) and interfaces with the linear programming solver (CPLEX).

`DVRPPricing.h`: Implements the pricing (labeling) algorithm for identifying routes with negative reduced costs.

`DVRPRoute.h`: Represents a route of the duration-constrained VRP and provides associated functions.

`DVRPSolution.h`: Represents a solution to the duration-constrained VRP (i.e., a set of routes).

`DVRPSolver.h`: Implements the petal heuristic to choose the best route combination among all routes created by column generation.

### Online Scheduling Policies (Online Policies)

`BasePolicy.h`: Abstract class (or *interface*) to be implemented by policies that are used as *base policies* within the rollout scheduling policy.

`Data.h`:

`Decision.h`:

`Dijkstra.h`:

`GreedyPolicy.h`:

`Instance.h`:

`Link.h`:

`MPotentialPlanner.h`:

`MaxFlowSolution.h`:

`MaxFlowSolver.h`:

`MinCutSolution.h`:

`MinCutSolver.h`:

`MyopicPlanner.h`:

`OfflinePlan.h`:

`OfflinePlanner.h`:

`OnlinePolicy.h`:

`PFAPolicy.h`:

`PGreedyPolicy.h`:

`Path.h`:

`PathTree.h`:

`PlannedRoute.h`:

`PolicySimulator.h`:

`PotentialPlanner.h`:

`Request.h`:

`RolloutPolicy.h`:

`RouteReoptimizer.h`:

`SimResults.h`:

`State.h`:

`Stopwatch.h`:

`TikZExporter.h`:

`mPGreedyPolicy.h`:

