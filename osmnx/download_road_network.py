# Road network analysis using osmnx
#
# Usage:
#  python download_road_network.py \
#     --city="Taipei, Taiwan" \
#     --visualize=False

from absl import app
from absl import flags
from os.path import exists
import logging
import matplotlib
import networkx as nx
import osmnx as ox


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string('city', 'Taipei, Taiwan', 'City for which to download.')
flags.DEFINE_bool('visualize', True, 'Whether to output downloaded graph as image')

logger = logging.getLogger('download_roald_network.py')

BASE_FILE_PATH = "./data/"


def main(argv):
  logger.info("Using osmnx version %s", ox.__version__)

  CITY_NAME = FLAGS.city.lower().replace(',', '').replace(' ', '_')
  GRAPHML_FILENAME = BASE_FILE_PATH + CITY_NAME + '.graphml'
  PNG_FILENAME = BASE_FILE_PATH + CITY_NAME + '.png'

  road_network = None
  if exists(GRAPHML_FILENAME):
    logger.info("Found existing graphml file %s ...", GRAPHML_FILENAME)
    road_network = ox.load_graphml(GRAPHML_FILENAME)
  else:
    logger.info("Start downloading road network...")
    road_network = ox.graph_from_place(FLAGS.city, network_type="drive")
    logger.info("Writing road network as grapml file %s", GRAPHML_FILENAME)
    ox.save_graphml(road_network, filepath=GRAPHML_FILENAME)

  if FLAGS.visualize:
    fig, ax = ox.plot_graph(
      road_network,
      show=False,
      filepath=PNG_FILENAME,
      save=True)

  nodes = road_network.nodes
  logger.info("Number of nodes: %d", len(nodes))
  logger.info("A node looks like: %s", nodes[7791704127])

  edges = road_network.edges
  logger.info("Number of edges: %d", len(edges))
  logger.info("An edge looks like: %s", edges[(9402576490, 2612435152, 0)])



if __name__ == '__main__':
  app.run(main)
