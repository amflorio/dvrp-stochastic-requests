# Dynamic Vehicle Routing with Stochastic Requests

## Description
The Dynamic Vehicle Routing Problem with Stochastic Requests (DVRPSR) is a prototypical problem in transportation logistics. The DVRPSR calls for an initial route plan and an online scheduling policy to route dynamically and in real-time a fleet of vehicles, in such a way that the number of customer requests accepted and served is maximized, and all vehicles return to the depot before a given deadline.

This repository contains the source code and datasets to allow the replication of the results from the paper:

**Zhang, J., Luo, K., Florio, A.M., & Van Woensel, T. (2022). Solving Large-Scale Dynamic Vehicle Routing Problems with Stochastic Requests.**

In particular, the code implements the following **scheduling policies** for the DVRPSR:
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
