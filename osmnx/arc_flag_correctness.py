"""
Checks the arc_flag algorithm correctness

Usage:
  python arc_flag_correctness --name=hcchao
"""

import logging
import utils
import random
import osmnx as ox
import pandas as pd
from absl import app
from absl import flags
from pathfinder import BoundingBox
from pathfinder import ArcFlagPathFinder

FLAGS = flags.FLAGS
flags.DEFINE_boolean('visualize', False, 'Whether to visualize the arc flag')
flags.DEFINE_integer('count', 10, 'Number of routes to compare')
flags.DEFINE_string(
  'region', 'Taipei Taiwan', 'Region name of road network to use')
flags.DEFINE_string('bbox', '25.033833,121.538415,25.024395,121.524799',
  'Comma separated bounding box in the order of north_lat,east_lon,south_lat,west_lon')

def sample_n_pairs(road_network, bounding_box, n):
  nodes_inside_region = []
  nodes_outside_region = []
  all_nodes = list(road_network.nodes.items())
  random.shuffle(all_nodes)
  # Sample N nodes in bbox
  for node_id, node_data in all_nodes:
    if len(nodes_inside_region) > n:
      break
    if bounding_box.contains(node_data['y'], node_data['x']):
      nodes_inside_region.append(node_id)
  if len(nodes_inside_region) < n:
    raise ValueError('Can\'t sample {} nodes inside bbox', n)
  # Sample N nodes outside bbox
  for node_id, node_data in all_nodes:
    if len(nodes_outside_region) > n:
      break
    if not bounding_box.contains(node_data['y'], node_data['x']):
      nodes_outside_region.append(node_id)
  if len(nodes_outside_region) < n:
    raise ValueError('Can\'t sample {} nodes outside bbox', n)
  pairs = zip(nodes_outside_region, nodes_inside_region)
  return pairs


def get_edge_colors(road_network):
  edge_colors = {}
  for edge_id, edge_data in road_network.edges.items():
    color = 'grey'
    try:
      if edge_data['R'] == True:
        color = 'r'
    except KeyError:
      pass
    edge_colors[edge_id] = color
  return pd.Series(data=edge_colors)


def main(argv):
  bbox_parts = [float(x) for x in FLAGS.bbox.split(',')]
  if len(bbox_parts) != 4:
    raise ValueError('Invalid bbox {}'.format(FLAGS.bbox))
  bounding_box = BoundingBox(
    bbox_parts[0], bbox_parts[2], bbox_parts[1], bbox_parts[3])

  road_network = utils.load_road_network(FLAGS.region, extension='graphml')
  af_pathfinder = ArcFlagPathFinder(road_network, bounding_box)

  if FLAGS.visualize:
    logging.info('Visualizing arc flags')
    ec = get_edge_colors(road_network)
    fig, ax = ox.plot_graph(
      road_network, edge_color=ec,
      edge_linewidth=1.0,
      dpi=1000, node_size=0)

  # randomply sample N pairs
  logging.info('Sampling %d pairs', FLAGS.count)
  random.seed(42)
  pairs = sample_n_pairs(road_network, bounding_box, FLAGS.count)

  all_equals = True
  for orig_id, dest_id in pairs:
    af_path, metadata = af_pathfinder.find_shortest_path(orig_id, dest_id)
    correct_path = ox.distance.shortest_path(road_network, orig_id, dest_id)
    if not utils.check_path_equals(af_path, correct_path):
      all_equals = False
  logging.info(
    '%s %d sampled routes return equal results',
    "All" if all_equals else "Not all", FLAGS.count)

if __name__ == '__main__':
  app.run(main)
