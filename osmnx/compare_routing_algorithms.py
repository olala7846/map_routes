# Compares differnet routing algorithms
#
# Usage:
#  python find_shortest_path.py

from absl import app
from absl import flags
from os.path import exists
from routing_algorithms import a_star_shortest_path
from routing_algorithms import dijkstra_shortest_path
import time
import logging
import osmnx as ox
import random
import numpy as np

# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string(
  "roadnetwork_file", "./data/greater_taipei.graphml",
  "Road network graph (graphml) file to use")

flags.DEFINE_integer("sample_count", 10, "Number of routes to compare")

logger = logging.getLogger("compare_routing_algorithms.py")



def _sample_n_pairs(graph):
  orig_dest_pairs = []
  num_nodes = len(graph.nodes)
  for i in range(FLAGS.sample_count):
    orig_id = list(graph.nodes.keys())[random.randint(0, num_nodes-1)]
    dest_id = list(graph.nodes.keys())[random.randint(0, num_nodes-1)]
    orig_dest_pairs.append((orig_id, dest_id))
  return orig_dest_pairs

def profile_algorithm(graph, orig_dest_pairs, algorithm_impl, weight="length"):
  took_time = []
  for orig_id, dest_id in orig_dest_pairs:
    t_start = time.process_time()
    algorithm_impl(graph, orig_id, dest_id, weight)
    t_end = time.process_time()
    took_time.append(t_end - t_start)
  return took_time


def main(argv):
  if not exists(FLAGS.roadnetwork_file):
    logger.error("Road network file %s not found", FLAGS.roadnetwork_file)
    return

  logger.info("Loading road network from file %s...", FLAGS.roadnetwork_file)
  G = ox.load_graphml(FLAGS.roadnetwork_file)

  random.seed(42)
  orig_dest_pairs = _sample_n_pairs(G)

  dijkstra_time = profile_algorithm(G, orig_dest_pairs, dijkstra_shortest_path)
  a_star_time = profile_algorithm(G, orig_dest_pairs, a_star_shortest_path)
  logger.info("Finish sampling %d routes.", FLAGS.sample_count)
  logger.info(
    "Dijkstra: mean(%f) std(%f) 95%%tile(%f)",
    np.mean(dijkstra_time), np.std(dijkstra_time), np.percentile(dijkstra_time, 95))
  logger.info(
    "A-Star: mean(%f) std(%f) 95%%tile(%f)",
    np.mean(a_star_time), np.std(a_star_time), np.percentile(a_star_time, 95))



if __name__ == '__main__':
  app.run(main)
