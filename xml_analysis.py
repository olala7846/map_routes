"""
xml_analysis.py
"""

from osm.parser import parse_xml
from osm.way import Way
from osm.geojson_util import way_to_geo_json
from geojson import FeatureCollection

OSM_FILE = 'taipei.osm'


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
  connected_nodes = set()
  for road in roads:
    connected_nodes |= set(road.get_node_ids())
  print('Number of Nodes connected by road: %s' % len(connected_nodes))

  def print_way(way):
    line_string = way_to_geo_json(way, nodes)
    print(line_string)

  # find '泰順街' and convert to GeoJSON format
  target_road = next(r for r in roads if r.get_name() == '泰順街')
  print(way_to_geo_json(target_road, nodes))

  # Generate road collection
  features = [way_to_geo_json(w, nodes) for w in roads]
  feature_collection = FeatureCollection(features)
  print('Collection:\n')
  print(feature_collection)
  # Visualizing this results shows that some single way lane are missing

  # Some statistic (e.g. avereage arcs per node)

  # Try draw a polyline on the map
  # https://developers.google.com/maps/documentation/javascript/reference/polygon#PolylineOptions


if __name__ == '__main__':
  main()
