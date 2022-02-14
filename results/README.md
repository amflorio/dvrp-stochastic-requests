## Complete Results

File `results.csv` contains the results of the 27,964 simulations conducted in the computational study in [1]. The fields indicate the following:

* **Rate**: Rate of requests in {0.2, 0.4, 0.8, 1.5}.
* **K**: Number of vehicles in {2, 3, 5, 6, 10, 12, 20}.
* **Dist**: Spatiotemporal request distribution in {UTI, CTI, CTD}.
* **Plan**: Offline plan in {pb, myo} (potential-based and myopic plans).
* **Policy**: Numerical value indicating the scheduling policy (matches the `<mode>` set when invoking `dvrpsr`).
* **Scenario**: Test trajectory in {1, 2, 3, 4, 5}.
* **Run**: Execution number in {1, ..., 10}. All probabilistic policies are simulated 10 times, so the results can be evaluated in expectation.
* **Accepted**: Number of dynamic requests accepted.
* **% Accepted**: Ratio of dynamic requests accepted over total number of dynamic requests.
* **Avg D Time**: Average time (in seconds) for taking a decision given a dynamic request.
* **Max D Time**: Maximum time (in seconds) for taking a decision given a dynamic request.
* **P80 D Time, P90 D Time, P95 D Time**: 80th, 90th and 95th percentiles of decision times.

