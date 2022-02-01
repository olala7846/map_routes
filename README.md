# Advance Routing Algorithm

## OSM data parser

A OSM (Open Street Map) data format parser. Parses the road network into a MultiDigraph (Could have multiple edges between two nodes, directed graph)
Project is located in [osm_parser](./osm_parser/README.md) folder.

* Data format: https://wiki.openstreetmap.org/wiki/OSM_XML
* Learning:
  * Xerces parser is very slow.
  * [osmnx](https://osmnx.readthedocs.io/) is an awesome library


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
