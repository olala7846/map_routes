# Compare A* having different heuristics
from absl import app
from absl import flags
from os.path import exists
from pathfinder import DijkstraPathFinder
from pathfinder import AStarPathFinder
from pathfinder import LandmarkPathFinder
from utils import load_road_network
import matplotlib.pyplot as plt
import time
import logging
import osmnx as ox
import random
import numpy as np
import networkx as nx
import pandas as pd
import utils


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string(
  'region', 'Greater Taipei', 'Region name of road network to use')
flags.DEFINE_integer(
  'count', 50, 'Number of pairs to compare')



def main(argv):
  road_network = load_road_network(FLAGS.region)

  random.seed(42)
  orig_dest_pairs = utils.sample_pairs(road_network, FLAGS.count)

  a_star_path_finder = AStarPathFinder(road_network)
  landmark_path_finder = LandmarkPathFinder(road_network, num_landmarks=30)

  real_costs = []
  euclid_costs = []
  landmark_costs = []

  for orig_id, dest_id in orig_dest_pairs:
    shortest_path = ox.distance.shortest_path(
      road_network, orig_id, dest_id, weight='length')
    if not shortest_path:
      continue
    real_costs.append(utils.calc_path_cost(shortest_path, road_network))
    euclid_costs.append(a_star_path_finder.heuristic_cost(orig_id, dest_id))
    landmark_costs.append(landmark_path_finder.heuristic_cost(orig_id, dest_id))
  logging.info('real cost size %d', len(real_costs))
  logging.info('eucl cost size %d', len(euclid_costs))
  logging.info('land cost size %d', len(landmark_costs))

  plt.scatter(real_costs, real_costs, c='grey', label='Real Cost')
  plt.scatter(real_costs, euclid_costs, c='g', label='Euclidean Dist')
  plt.scatter(real_costs, landmark_costs, c='b', label='Landmark')

  plt.xlabel('Real Cost')
  plt.ylabel('Heuristic')
  plt.legend(loc='upper left')
  plt.show()


if __name__ == '__main__':
  app.run(main)
