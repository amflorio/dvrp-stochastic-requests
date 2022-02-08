## Input Files

File `coords/vienna.xy` contains the `(x,y)` coordinates of the road intersections of the street network of Vienna (Austria). These coordinates maintain the proportions of the equirectangular projection of the latitude and longitude coordinates of each node. Therefore, they can be used to create images to represent the network.

File `dists/vienna.d` describes the road segments of the network. The first line indicates the number of road segments (or arcs) in the network. Each following line has the format `<o> <d> <l>` and determine the origin, destination and length (in meters) of each arc.
