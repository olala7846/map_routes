"""
xml_analysis.py
"""
from collections import defaultdict, deque
from geojson import FeatureCollection, dumps
from osm.parser import parse_xml
from osm.way import Way
from osm.geojson_util import way_to_geo_json


OSM_FILE = 'taipei.osm'
OUT_FILE = './out/roads.geojson'


def main():
  '''main function'''
  nodes, ways = parse_xml(OSM_FILE)


  # Try to answer the following questions:
  # How many Nodes & Ways
  print('Number of nodes: {}, Number of ways: {}'.format(len(nodes), len(ways)))

  # How many Ways are roads (have highyway tag)
  def has_type(way: Way):
    way_type = way.get_type()
    return not way_type is None

  roads = list(filter(has_type, ways.values()))
  print('Number of ways that are road: %s' % len(roads))

  # How many Nodes are connected to a real road
  # This is a rough estimation by filtering ways with highway tag
  connected_nodes = set()
  for road in roads:
    connected_nodes |= set(road.get_node_ids())
  print('Number of Nodes connected by road: %s' % len(connected_nodes))

  # Do a real connected component traverse.
  # build adjcency metrics
  adjcency_metrics = build_adjaceny_metric(ways)
  num_connected_components: int = 0
  unexplored = set(nodes.keys())
  exploring = set()
  exploited = set()
  while True:
    try:
      seed = unexplored.pop()
      num_connected_components += 1
      exploring.add(seed)
    except KeyError:
      break # unexplored set is empty

    while True:
      try:
        elem = exploring.pop()
      except KeyError:
        break # nothing else to explore

      exploited.add(elem)
      neighbour_nodes = adjcency_metrics[elem]
      unexplored -= neighbour_nodes
      exploring |= (neighbour_nodes - exploited)
  print ('Number of connected components %s' % num_connected_components)

  # find '泰順街' and convert to GeoJSON format
  target_road = next(r for r in roads if r.get_name() == '泰順街')
  print(way_to_geo_json(target_road, nodes))

  # Generate road collection
  features = [way_to_geo_json(w, nodes) for w in roads]
  feature_collection = FeatureCollection(features)
  with open(OUT_FILE, "w") as out_file:
    out_file.write(dumps(feature_collection))

  # Visualizing this results shows that some single way lane are missing

  # Some statistic (e.g. avereage arcs per node)

  # Try draw a polyline on the map
  # https://developers.google.com/maps/documentation/javascript/reference/polygon#PolylineOptions


def build_adjaceny_metric(ways):
  # dictionary<node_id, > of set
  adjcency_metrics = defaultdict(set)
  for way in ways.values():
    # for each arc, add adjency metircs for two direction
    # TODO(hcchao): handle single way lane
    arc_from = None
    for arc_to in way.get_node_ids():
      if arc_from is None:
        arc_from = arc_to
        continue
      else:
        adjcency_metrics[arc_from].add(arc_to)
        adjcency_metrics[arc_to].add(arc_from)
  return adjcency_metrics

if __name__ == '__main__':
  main()
