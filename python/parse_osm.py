# OSM file parser
#
# Usage:
#  python parse_osm.py \
#     --filename=data/taipei.osm \
#     --output_geojson

import logging
import random
import xml.etree.ElementTree as ET
import heapq

import geopy.distance
from absl import app
from absl import flags
from exceptions import OsmError
from geojson import Feature, FeatureCollection, MultiLineString, LineString
from geojson import dumps


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

  def __str__(self):
    return 'OsmNode(id={}, lat={}, lon={})'.format(
      self.node_id, self.lat, self.lon)

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
  logger.info('Will start parsing "%s"', filename)
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

  # Remove nodes that are not connected by the road network
  all_node_ids = set(road_network.keys())
  connected_node_ids = set()
  for node_id, node in road_network.items():
    if len(node.arcs) > 0:
      connected_node_ids.add(node_id)
      for dest_node_id, _ in node.arcs:
        connected_node_ids.add(dest_node_id)
  unconnected_nodes = all_node_ids - connected_node_ids
  for node_id in unconnected_nodes:
    del road_network[node_id]
  logger.info(
    'Deleted %d unconnected nodes, %d left',
    len(unconnected_nodes), len(connected_node_ids))


  if FLAGS.output_geojson:
    out_filename_parts = FLAGS.filename.split('.')
    out_filename_parts[-1] = 'geojson'  # replace file extension to geojson
    feature_collection = road_network_to_feature_collection(road_network)
    with open('.'.join(out_filename_parts), 'w') as out_file:
      out_file.write(dumps(feature_collection))

  return road_network


def dijkstra_search(src_id, dest_id, road_network):
  frontier = [(0, src_id, src_id)]  # (cost, node_id, origin_node_id)
  explored = set()
  source_map = dict()  # records where each node was from

  while frontier:
    curr_cost, curr_node_id, origin_node_id = heapq.heappop(frontier)

    if curr_node_id in explored:  # already there
      continue
    # mark node as explored and record the origin when first time reaching the node
    explored.add(curr_node_id)
    source_map[curr_node_id] = origin_node_id

    if curr_node_id == dest_id:  # found
      path_rev = []
      path_node_id = dest_id
      while path_node_id != source_map[path_node_id]:
        path_rev.append(path_node_id)
        path_node_id = source_map[path_node_id]
      path_rev.append(src_id)
      path_rev.reverse()
      return path_rev

    curr_node = road_network[curr_node_id]
    for arc_dest, arc_cost in curr_node.arcs:
      heapq.heappush(frontier, (curr_cost + arc_cost, arc_dest, curr_node_id))


def draw_route_found(route, road_network):
  line_string = []
  for node_id in route:
    osm_node = road_network[node_id]
    line_string.append((osm_node.lon, osm_node.lat))
  features = [Feature(geometry=LineString(line_string))]
  feature_collection = FeatureCollection(features)
  with open('data/route_found.geojson', 'w') as out_file:
    out_file.write(dumps(feature_collection))


def main(argv):
  road_network = parse_osm_xml(FLAGS.filename)
  random.seed(42)
  node_ids = list(road_network.keys())
  src_id = random.choice(node_ids)
  dest_id = random.choice(node_ids)
  logger.info(
      'Find route from %s to %s.',
      road_network[src_id], road_network[dest_id])

  route = dijkstra_search(src_id, dest_id, road_network)
  logger.info('Returned route is: %s', route)

  draw_route_found(route, road_network)


if __name__ == '__main__':
  app.run(main)
