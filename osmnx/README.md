# OSM nx
 Try download OSM road network using [OSMnx](https://osmnx.readthedocs.io/en/stable/) and test different routing algorithm

 * Dijkstra
 * Two-way Dijkstra
 * A-Star
 * Landmark
 * Contractino heirarchy


## OSMnx

Build upon [GeoPandas](https://geopandas.org/en/stable/) and [NetworkX](https://networkx.org/)

## GraphML
http://graphml.graphdrawing.org/primer/graphml-primer.html

### Resource
* https://osmnx.readthedocs.io/en/stable/
* https://github.com/gboeing/osmnx-examples/ See notebooks

### Set up instruction
[Installation guide](https://osmnx.readthedocs.io/en/stable/) or `pip install osmnx`


## Learnings

When comparing Dijkstra and A-Star in a city area, A-Star is not always faster. For example, for shortest route at about 60 nodes, Dijkstra explored 6555 nodes, A-star explored 1704 nodes. But each step calculating A_star cost is more than 6 times expensive then Dijkstra.

To verify this assumption, we need larger city in order to test it..

### Algorithm

#### Dijkstra

Dijkstra (and also A*) utilaize a priority queue to find the lowest cost in the frontier.
Heap (used as a priortiy queue) don't usually support DecreaseKey (or updateKey) operation.
We simply append a new set (new_cost, node) into the heap and ignore any already explored
(settled) keys in the while loop in Dijkstra. This is pretty much as efficient due to the
nature of road network (almost a planer graph).

#### A*

* Euclidiance distance heuristic
* Landmark heuristic
    For any node u, v, landmark l

    ```
    dist(u,l) <= dist(u,v) + dist(v,l)  // by monotone
    dist(l,v) <= dist(l,u) + dist(u,v)  // by monotone
    ```

    The above inequality and be re-written as

    ```
    dist(u,l) - dist(v,l) <= dist(u,v)
    dist(l,v) - dist(l,u) <= dist(u,v)
    ```
    Hence by combining the two we get the landmark heuristic

    ```
    h(u,v) == max( dist(u,l)-dist(v,l), dist(l,v)-dist(l,u) ) <= dist(u.v)
    ```

