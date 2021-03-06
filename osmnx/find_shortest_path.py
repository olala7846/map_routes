# Find shortest path and visualize
#
# Usage:
#  python find_shortest_path.py \
#     --roadnetwork_file="./data/taiwan.graphml" \
#     --destination="25.1240308,121.6475832" \
#     --algorithm=a_star
#     --visualize

import logging

from absl import app
from absl import flags
from os.path import exists
import networkx as nx
import osmnx as ox
import time
import matplotlib
from pathfinder import DijkstraPathFinder
from pathfinder import AStarPathFinder
from pathfinder import LandmarkPathFinder


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS


flags.DEFINE_string(
  "roadnetwork_file", "./network/greater_taipei.graphml",
  "Road network graph (graphml) file to use")

# 25.0372619,121.554578 台北市政府
# 22.6091931,120.2946914 高雄市政府

flags.DEFINE_string(
  "origin", "25.0854479,121.5201535",
  "Route origin lat,lon")
flags.DEFINE_string(
  "destination", "25.0382115,121.5533113",
  "Route destination lat,lon")

flags.DEFINE_bool("visualize", False, "whether to visualize the route or not.")

flags.DEFINE_enum("algorithm", "dijkstra", [
  "dijkstra", "a_star", "landmark",
  "two_way_dijkstra", "CH"
],
"Algorithm to find shortest path")

logger = logging.getLogger("find_shortest_path.py")



def _route_equals(route1, route2):
  if len(route1) != len(route2):
    logger.error("Route length different %d vs %d", len(route1), len(route2))
    return False
  for n1, n2 in zip(route1, route2):
    if n1 != n2:
      return False
  return True


def main(argv):
  logger.info("Using osmnx version %s", ox.__version__)

  if not exists(FLAGS.roadnetwork_file):
    logger.error("Road network file %s not found", FLAGS.roadnetwork_file)
    return

  logger.info("Loading road network from file %s", FLAGS.roadnetwork_file)
  G = ox.load_graphml(FLAGS.roadnetwork_file)

  orig_lat, orig_lon = [float(x) for x in FLAGS.origin.split(',')]
  dest_lat, dest_lon = [float(x) for x in FLAGS.destination.split(',')]

  orig_id = ox.distance.nearest_nodes(G, X=orig_lon, Y=orig_lat)
  dest_id = ox.distance.nearest_nodes(G, X=dest_lon, Y=dest_lat)
  logger.info("origin- id: %d, lat: %f, lon: %f", orig_id, orig_lat, orig_lon)
  logger.info("destination- id:%d, lat: %f, lon: %f", dest_id, dest_lat, dest_lon)


  t_start = time.process_time()
  osmnx_route = ox.shortest_path(G, orig_id, dest_id, weight="length")
  t_end = time.process_time()
  logger.info("OSMnx routes took %f seconds", t_end - t_start)

  algorithm = FLAGS.algorithm
  path_finder = DijkstraPathFinder(G)
  if algorithm == 'a_star':
    logger.info("Using A* Distance Heuristic")
    path_finder = AStarPathFinder(G)
  if algorithm == 'landmark':
    logger.info("Using Landmark Heuristic")
    path_finder = LandmarkPathFinder(G)

  else:
    logger.info("Using DijkstraPathFinder")

  t_start = time.process_time()
  route = path_finder.find_shortest_path(orig_id, dest_id)
  t_end = time.process_time()
  logger.info("%s routes took %f seconds", path_finder.get_name(), t_end - t_start)

  route_equals = _route_equals(osmnx_route, route)
  logger.info("Route equals: %s", route_equals)
  if not route_equals:
    logger.error("The route found is not the shortest path.")
  if not route_equals or FLAGS.visualize:
    logger.info("Visualizing...")
    fig, ax = ox.plot_graph_routes(
      G, [osmnx_route, route], route_colors=['r', 'g'],
      route_linewidths=2, orig_dest_size=4,
      filepath="./data/routes.png", save=True, edge_linewidth=0.5,
      dpi=1000, node_size=1)



if __name__ == '__main__':
  app.run(main)
