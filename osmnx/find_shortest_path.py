# Find shortest path and visualize
#
# Usage:
#  python find_shortest_path.py

from absl import app
from absl import flags
from os.path import exists
from collections import defaultdict
import logging
import matplotlib
import networkx as nx
import osmnx as ox
import time
import heapq


# ABSL flags: https://abseil.io/docs/python/guides/flags
FLAGS = flags.FLAGS


flags.DEFINE_string(
  'roadnetwork_file', './data/taipei_taiwan.graphml',
  'Road network graph (graphml) file to use')

flags.DEFINE_string(
  'origin', '25.0854479,121.5201535',
  'Route origin lat,lon')
flags.DEFINE_string(
  'destination', '25.0382115,121.5533113',
  'Route destination lat,lon')

logger = logging.getLogger('find_shortest_path.py')


def _construct_route(is_from, orig_id, dest_id):
  route = []
  node_id = dest_id
  while node_id != orig_id:
    route.insert(0, node_id)
    node_id = is_from[node_id]
  route.insert(0, orig_id)
  return route


def dijkstra_shortest_path(graph, orig_id, dest_id, weight="length"):
  explored = set()
  is_from = dict()
  min_cost = defaultdict(lambda : float('inf'))
  frontier = [(0.0, orig_id)]  # (cost, id)
  while len(frontier) > 0:
    curr_cost, curr_node_id = heapq.heappop(frontier)
    if curr_node_id == dest_id:
      return _construct_route(is_from, orig_id, dest_id)
    explored.add(curr_node_id)
    for next_node_id, edge in graph[curr_node_id].items():
      if next_node_id in explored:
        continue
      else:
        new_cost = curr_cost + edge[0][weight]
        if min_cost[next_node_id] > new_cost:
          min_cost[next_node_id] = new_cost
          is_from[next_node_id] = curr_node_id
          heapq.heappush(frontier, (new_cost, next_node_id))
  return None

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

  logger.info("Start downloading road network...")
  G = ox.load_graphml(FLAGS.roadnetwork_file)

  orig_lat, orig_lon = [float(x) for x in FLAGS.origin.split(',')]
  dest_lat, dest_lon = [float(x) for x in FLAGS.destination.split(',')]

  orig_id = ox.distance.nearest_nodes(G, X=orig_lon, Y=orig_lat)
  dest_id = ox.distance.nearest_nodes(G, X=dest_lon, Y=dest_lat)
  logger.info("origin- id: %d, lat: %f, lon: %f", orig_id, orig_lat, orig_lon)
  logger.info("destination- id:%d, lat: %f, lon: %f", dest_id, dest_lat, dest_lon)

  t_start = time.process_time()
  route = ox.shortest_path(G, orig_id, dest_id, weight="length")
  t_end = time.process_time()
  logger.info("Route1 %s", route)
  logger.info("OSMnx routes took %f seconds", t_end - t_start)
  # fig, ax = ox.plot_graph_route(G, route, node_size=0)

  t_start = time.process_time()
  route2 = dijkstra_shortest_path(G, orig_id, dest_id, weight="length")
  t_end = time.process_time()
  logger.info("Route2 %s", route2)
  logger.info("dijkstra routes took %f seconds", t_end - t_start)
  fig, ax = ox.plot_graph_routes(
    G, [route, route2], route_colors=['r', 'g'],
    route_linewidths=2, orig_dest_size=4,
    filepath="./data/routes.png", save=True, edge_linewidth=0.5,
    dpi=1000, node_size=1)

  logger.info("Route equals: %s", _route_equals(route, route2))


if __name__ == '__main__':
  app.run(main)
