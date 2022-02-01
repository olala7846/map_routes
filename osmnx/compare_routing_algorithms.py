# Compares differnet routing algorithms
#
# Usage:
#  python compare_routing_algorithm.py --count=50 --plot=true

from absl import app
from absl import flags
from os.path import exists
from pathfinder import DijkstraPathFinder
from pathfinder import AStarPathFinder
from pathfinder import LandmarkPathFinder
import utils
import matplotlib.pyplot as plt
import time
import logging
import osmnx as ox
import random
import numpy as np
import networkx as nx
import pandas as pd


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string(
  'region', 'Greater Taipei', 'Region name of road network to use')

flags.DEFINE_integer('count', 10, 'Number of routes to compare')
flags.DEFINE_bool('plot', True, 'To plot the profile results or not.')
flags.DEFINE_enum(
  'y_axis', 'time', ['time', 'settled'],'Value to use for compare (y-axis).')

logger = logging.getLogger('compare_routing_algorithms.py')

def distance_between(p1_id : str, p2_id : str, road_network : nx.Graph):
  """Calculates the distance between two nodes."""
  p1 = road_network.nodes[p1_id]
  p2 = road_network.nodes[p2_id]
  p1_lat, p1_lon = p1['y'], p1['x']
  p2_lat, p2_lon = p2['y'], p2['x']
  return ox.distance.great_circle_vec(p1_lat, p1_lon, p2_lat, p2_lon)


def profile_algorithm(pathfinder, orig_dest_pairs):
  """Profile pathfinder performance accorss all origin/destination paris.

  Returns:
    A dataframe with all the results, having 'distance' and 'time' columns
  """
  all_distance = []
  all_time = []
  all_settled_cnt = []
  for orig_id, dest_id in orig_dest_pairs:
    t_start = time.process_time()
    unused_route, metadata = pathfinder.find_shortest_path(orig_id, dest_id)
    t_end = time.process_time()
    used_time = t_end - t_start
    distance = distance_between(orig_id, dest_id, pathfinder.road_network)
    all_distance.append(distance)
    all_time.append(used_time)
    all_settled_cnt.append(metadata['settled'])

  return pd.DataFrame(data={
    'distance': all_distance,
    'time': all_time,
    'settled': all_settled_cnt})

def get_road_network_by_name(file_name):
  if not exists(FLAGS.roadnetwork_file):
    logger.error("Road network file %s not found", file_name)
    return

  logger.info("Loading road network from file %s...", file_name)
  return ox.load_graphml(FLAGS.roadnetwork_file)


def main(argv):
  road_network = utils.load_road_network(FLAGS.region)

  random.seed(42)
  orig_dest_pairs = utils.sample_pairs(road_network, FLAGS.count)

  dijkstra_path_finder = DijkstraPathFinder(road_network)
  a_star_path_finder = AStarPathFinder(road_network)
  landmark_path_finder = LandmarkPathFinder(road_network, num_landmarks=30)

  path_finders_to_compare = {
    'r': dijkstra_path_finder,
    'g': a_star_path_finder,
    'b': landmark_path_finder,
  }

  y_axis = FLAGS.y_axis  # 'time' or 'settled'
  for color, pathfinder in path_finders_to_compare.items():
    report = profile_algorithm(pathfinder, orig_dest_pairs)

    if FLAGS.plot:
      plt.scatter(report['distance'], report[y_axis], c=color, label=pathfinder.get_name())

  if FLAGS.plot:
    plt.xlabel('Distsance (meter)')
    plt.ylabel(y_axis)
    plt.legend(loc='upper left')
    plt.show()


if __name__ == '__main__':
  app.run(main)
