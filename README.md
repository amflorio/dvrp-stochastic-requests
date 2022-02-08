# Dynamic Vehicle Routing with Stochastic Requests

## Description
The Dynamic Vehicle Routing Problem with Stochastic Requests (DVRPSR) is a prototypical problem in transportation logistics. The DVRPSR calls for an initial route plan and an online scheduling policy to route dynamically and in real-time a fleet of vehicles, in such a way that the number of customer requests accepted and served is maximized, and all vehicles return to the depot before a given deadline.

This repository contains the source code and datasets to allow the replication of the results from the paper:

**Zhang, J., Luo, K., Florio, A.M., & Van Woensel, T. (2022). Solving Large-Scale Dynamic Vehicle Routing Problems with Stochastic Requests.**

In particular, the source code implements the following scheduling policies for the DVRPSR:
* Greedy policies: accepts dynamic requests in a greedy fashion, and update planned routes either by cheapest insertion or by complete reoptimization.
* PFA-based policy: accepts dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to-go, which is estimated by policy function approximation.
* Rollout policies, to be used with the Greedy and PFA-based policies as base policies.
* Potential-based policy: accepted dynamic requests as long as the immediate reward offsets the estimated decrease in the reward-to
* Rollout policies (with and without reoptimization): 
* 

In addition, code for the following offline planners is also provided:
* Myopic planner: an offline route planner that solves approximately a duration-constrained VRP by a column generation-based heuristic.
* Potential-based planner: an offline route planner that evaluates the expected reward-to-go of planned routes by the single-knapsack potential approximation

## Dependencies

## Usage

## Documentation

