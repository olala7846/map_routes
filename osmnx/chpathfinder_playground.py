"""
CHPathFinder playground

Usage:
  python  --name=hcchao
"""

import logging
import os
import networkx as nx
import osmnx as ox
import time
import matplotlib

from absl import app
from absl import flags
from pathfinder import CHPathFinder

FLAGS = flags.FLAGS
flags.DEFINE_string(
  "roadnetwork_file", "./network/taipei_taiwan.graphml",
  "Road network graph (graphml) file to use")
flags.DEFINE_bool(
  "plot_ch_time", False, "Plots the time used to contract node")
flags.DEFINE_bool(
  "plot_edge_diff", True, "Plots the contraction edge diff")

logger = logging.getLogger("chpathfinder_playground.py")

def main(argv):
  if not os.path.exists(FLAGS.roadnetwork_file):
    logger.error("Road network file %s not found", FLAGS.roadnetwork_file)
    return

  logger.info("Loading road network from file %s", FLAGS.roadnetwork_file)
  MG = ox.load_graphml(FLAGS.roadnetwork_file)

  pathfinder = CHPathFinder(
    MG, build_on_init=False, num_nodes_to_contract=2000)
  time_spent = []
  edge_diff = []

  max_t = 0.0
  for node_id in pathfinder.random_node_ordering():
    t_start = time.process_time()
    num_edge_added, num_edge_deleted = pathfinder.contract_node(node_id)
    t_end = time.process_time()
    time_spent_sec = t_end - t_start
    if time_spent_sec > max_t:
      max_t = time_spent_sec
    time_spent.append(time_spent_sec)
    edge_diff.append(num_edge_added - num_edge_deleted)

  if FLAGS.plot_ch_time:
    matplotlib.pyplot.hist(
      time_spent,
      'auto', # bins
      (0.0, max_t * 1.1), #range
      log=True,)
    matplotlib.pyplot.show()

  if FLAGS.plot_edge_diff:
    matplotlib.pyplot.hist(
      edge_diff,
      100, # bins
      (min(edge_diff), max(edge_diff)), #range
      log=True,)
    matplotlib.pyplot.show()




if __name__ == '__main__':
  app.run(main)
