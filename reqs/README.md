## Request Files

This folder contains the request trajectories used in the computational experiments.

The format of each file is `V-<Lambda>-<dist>.<n>.req`, where:

`Lambda` is the rate of dynamic requests (requests per minute),\
`dist` in {UTI, CTI, CTD} is the distribution of dynamic requests, and\
`n` in {1,2,3,4,5} is the trajectory number.

Please refer to the paper for more details about the parameters.

Each trajectory file consists of a sequence of triples `(u,i,d)`, where:

`u` is the request arrival time (`u=0` for static or planned requests, `u>0` for dynamic requests),\
`i` is the request node, and\
`d` is the time required to serve the request.
