# OSM data parser

Data: taipei.osm
Data format: https://wiki.openstreetmap.org/wiki/OSM_XML

# Utility Scripts
* xml_analysis.py: count number of Nodes, Way ... in the OSM file
  ```
  $ python3 xml_analysis.py

  defaultdict(<class 'int'>, {'bounds': 1, 'node': 44616, 'way': 6687, 'relation': 572})
  ```


# Data Format

Data can be converted into GeoJSON data (https://geojson.org/)
`pip3 install geojson`
and can be visualized using web UI tool https://geojson.io/

# Visualization App

/angular-maps-api-demo contains an Angular 2 App for visualizing data
use `ng serve` to start app locally
see /angular-maps-api-demo/README.md
