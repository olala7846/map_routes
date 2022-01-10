# OSM file parser
#
# Usage:
#  python parse_osm.py

import logging
import math
import xml.etree.ElementTree as ET

import geopy.distance
from absl import app
from absl import flags
from exceptions import OsmError
from geojson import Feature, FeatureCollection, MultiLineString, dumps


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string('filename', 'data/taipei_daan.osm', 'OSM file to be parsed.')
flags.DEFINE_bool(
  'output_geojson', False,
  'Wheather to output the parsed road network as a GeoJSON fole for not')

logger = logging.getLogger('parse_osm.py')

# https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed
# Speed in meter per seconds
SPEED_MAP = {
  'motorway': 31.2928,       # 70.0 mph
  'motorway_link': 31.2928,  # 70.0 mph
  'trunk': 24.5872,          # 55.0 mph
  'trunk_link': 20.1168,     # 45.0 mph
  'primary': 24.5872,        # 55.0 mph
  'primary_link': 24.5872,   # 55.0 mph
  'secondary': 20.1168,      # 45.0 mph
  'secondary_link': 20.1168, # 45.0 mph
  'tertiary': 15.6464,       # 35.0 mph
  'tertiary_link': 15.6464,  # 35.0 mph
  'unclassified': 24.5872,   # 55.0 mph
  'residential': 11.176,     # 25.0 mph
  'living_street': 11.176,   # 25.0 mph
  'service': 11.176,         # 25.0 mph
}
EARTH_R = 6378.137  # Radius of earth in KM

class OsmNode:

  def __init__(self, node_id, lat, lon):
    self.node_id = node_id
    self.lat = lat
    self.lon = lon
    self.arcs = []  # tuple of (destination_id, and cost)

  def add_arc(self, dest_id, cost):
    self.arcs.append((dest_id, cost))


def parse_node(xml_node):
  attributes = xml_node.attrib
  node_id = attributes['id']
  return node_id, OsmNode(node_id, float(attributes['lat']), float(attributes['lon']))


def parse_way(xml_way):
  """Returns a iterable of edges."""
  way_tags = dict()
  for tag in xml_way.iter('tag'):
    way_tags[tag.attrib['k']] = tag.attrib['v']

  highway_type = None
  try:
    highway_type = way_tags['highway']
  except KeyError:
    return []

  is_oneway = False  # default two way
  try:
    is_oneway = True if way_tags['oneway'] == 'yes' else False
  except KeyError:
    pass  # no oneway tag specified

  path = [nd.attrib['ref'] for nd in xml_way.iter('nd')]
  for i in range(len(path) - 1):
    src_id = path[i]
    dest_id = path[i+1]
    try:
      speed_limit = SPEED_MAP[highway_type]
    except KeyError:
      # Unlisted road types are considered non drivable
      speed_limit = 5.0
    yield (src_id, dest_id, speed_limit)
    if not is_oneway:
      yield (dest_id, src_id, speed_limit)


def calculate_cost(src_node, dest_node, speed_limit):
  src_coord = (src_node.lat, src_node.lon)
  dest_coord = (dest_node.lat, dest_node.lon)
  # https://en.wikipedia.org/wiki/Haversine_formula
  distance = geopy.distance.geodesic(src_coord, dest_coord).km * 1000
  return distance / speed_limit


def road_network_to_feature_collection(road_network: dict):
  def get_lonlat(road_network, node_id):
    node = road_network[node_id]
    return (node.lon, node.lat)

  features = []
  for node_id, osm_node in road_network.items():
    lines = []
    origin_lonlat = get_lonlat(road_network, node_id)
    for dest_id, _ in osm_node.arcs:
      dest_lonlat = get_lonlat(road_network, dest_id)
      lines.append([origin_lonlat, dest_lonlat])
    if not lines:  # skip nodes without arcs
      continue

    multi_line_string = MultiLineString(lines)
    features.append(Feature(geometry=multi_line_string))
    # force break for testing
  logger.info('Total %d features in collection.', len(features))

  return FeatureCollection(features)

def parse_osm_xml(filename):
  tree = ET.parse(filename)
  root = tree.getroot()
  if root.tag != 'osm':
    raise OsmError('root node must be of tag osm')

  road_network = dict()

  for child in root:
    if child.tag == 'node':
      node_id, node = parse_node(child)
      road_network[node_id] = node
    elif child.tag == 'way':
      way_iter = parse_way(child)
      for src_id, dest_id, speed_limit in way_iter:
        src_node = road_network[src_id]
        dest_node = road_network[dest_id]
        cost = calculate_cost(src_node, dest_node, speed_limit)
        src_node.add_arc(dest_id, cost)

  # parse finish, print statistics
  num_nodes = len(road_network.values())
  nodes_with_road = list(filter(lambda nd: len(nd.arcs) > 0, iter(road_network.values())))

  logger.info('there are total %d nodes, %d of them are connected to roads',
        num_nodes, len(nodes_with_road))

  if FLAGS.output_geojson:
    out_filename_parts = FLAGS.filename.split('.')
    out_filename_parts[-1] = 'geojson'  # replace file extension to geojson
    feature_collection = road_network_to_feature_collection(road_network)
    with open('.'.join(out_filename_parts), 'w') as out_file:
      out_file.write(dumps(feature_collection))



def main(argv):
  logger.info('Will start parsing "%s"', FLAGS.filename)

  parse_osm_xml(FLAGS.filename)


if __name__ == '__main__':
  app.run(main)
