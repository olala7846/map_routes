# Road network analysis using osmnx
#
# Usage
#  Download graph by city name:
#    python download_road_network.py \
#       --city="Taipei, Taiwan" \
#       --visualize=False
#  Download graph by bounding box
#    python download_road_network.py \
#       --bbox="25.3310,24.8178,122.0332,121.2567" \
#       --bbox_name="great_taipei_area"
#

from absl import app
from absl import flags
from os.path import exists
import logging
import matplotlib
import networkx as nx
import osmnx as ox
from exceptions import FlagException


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS

flags.DEFINE_string('city', None, 'City for which to download.')
flags.DEFINE_string('bbox', None, 'Bounding box in north,south,east,west order')
flags.DEFINE_string('bbox_name', None, 'Bounding box name')
flags.DEFINE_bool('visualize', True, 'Whether to output downloaded graph as image')

logger = logging.getLogger('download_roald_network.py')

BASE_FILE_PATH = "./data/"
BASE_NETWORK_PATH = "./network/"



def main(argv):
  logger.info("Using osmnx version %s", ox.__version__)

  road_network = None
  if FLAGS.bbox and FLAGS.city:
    raise FlagException("Can't have both --bbox and --city being set")

  graphml_filename = None
  image_filename = None
  if FLAGS.city:
    city_name = FLAGS.city.lower().replace(',', '').replace(' ', '_')
    image_filename = BASE_FILE_PATH + city_name + '.png'
    graphml_filename = BASE_FILE_PATH + city_name + '.graphml'
  elif FLAGS.bbox:
    bbox_name = FLAGS.bbox_name if FLAGS.bbox_name else 'bbox'
    image_filename = BASE_FILE_PATH + bbox_name + '.png'
    graphml_filename = BASE_FILE_PATH + bbox_name + '.graphml'

  gpickle_filename = BASE_NETWORK_PATH + city_name + '.gpickle'

  if exists(gpickle_filename):
    logger.info("Found existing gpickle file %s ...", gpickle_filename)
    road_network = nx.read_gpickle(gpickle_filename)
  elif exists(graphml_filename):
    logger.info("Found existing graphml file %s ...", graphml_filename)
    road_network = ox.load_graphml(graphml_filename)
  elif FLAGS.city:
    logger.info("Start downloading road network...")
    road_network = ox.graph_from_place(FLAGS.city, network_type="drive")
    logger.info("Writing road network as grapml file %s", graphml_filename)
    ox.save_graphml(road_network, filepath=graphml_filename)
  elif FLAGS.bbox:
    logger.info("Start downloading road network...")
    parts = FLAGS.bbox.split(',')
    if len(parts) != 4:
      raise FlagException("--bbox=\"<north>,<south>,<east>,<west>\"")
    numbers = [float(n) for n in parts]
    road_network = ox.graph_from_bbox(
      numbers[0], numbers[1], numbers[2], numbers[3], network_type="drive")
    logger.info("Writing road network as grapml file %s", graphml_filename)
    ox.save_graphml(road_network, filepath=graphml_filename)
  else:
    logger.error("Unexpected error.")
    return

  if FLAGS.visualize:
    fig, ax = ox.plot_graph(
      road_network,
      show=False,
      filepath=image_filename,
      save=True,
      edge_linewidth=0.5,
      dpi=1000, node_size=1)

  nodes = road_network.nodes
  logger.info("Number of nodes: %d", len(nodes))
  logger.info("A node looks like: %s", nodes[7791704127])

  edges = road_network.edges
  logger.info("Number of edges: %d", len(edges))
  logger.info("An edge looks like: %s", edges[(9402576490, 2612435152, 0)])

  # Test writing file as NextworkX gpickle file
  # https://networkx.org/documentation/stable/reference/readwrite/gpickle.html
  if not exists(gpickle_filename):
    logging.info("%s not found, create one", gpickle_filename)
    nx.write_gpickle(road_network, gpickle_filename)



if __name__ == '__main__':
  app.run(main)
