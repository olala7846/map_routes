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
