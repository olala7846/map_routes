# Advance Routing Algorithm

## OSM data parser

A OSM (Open Street Map) data format parser. Parses the road network into a MultiDigraph (Could have multiple edges between two nodes, directed graph)

* See [osm_parser](./osm_parser/README.md) folder for detailed instruction and learnings.

* Data format: https://wiki.openstreetmap.org/wiki/OSM_XML


## Routing Algorithm
Implement different routing (shortest path) algorithm and comparse the results.
See [osmnx](./osmnx/README.md) filder for more.

* Algorithms Dijkstra, A* (different heuristics).
* Arc flag (TODO)
* Contraction Heirarchy (TODO)



## Legacy Projects

### Visualization App

/angular-maps-api-demo contains an Angular 2 App for visualizing data
use `ng serve` to start app locally
see [/angular-maps-api-demo/README.md](./angular-maps-api-demo/README.md)
