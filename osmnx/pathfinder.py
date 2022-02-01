"""Path finder algorithm library"""

from collections import defaultdict
import heapq
import math
import numpy
import networkx as nx
import logging


EARTH_RADIUS=6371009


def _reconstruct_route(origin_id, destination_id, is_from):
  """Construct full route from origin to destination."""
  route = list()
  node_id = destination_id
  while node_id != origin_id:
    route.insert(0, node_id)
    node_id = is_from[node_id]
  route.insert(0, origin_id)
  return route


class PathFinderInterface():
  """Base pathfinder interface."""

  def get_name(self) -> str:
    """Returns the human readable name of the pathfinder."""
    return NotImplementedError("Must implement get_name")

  def find_shortest_path(self, origin_id : str, destination_id : str):
    """Returns the shortest path from origin to destination."""
    raise NotImplementedError("Must implement find_shortest_path")


class BestFirstSearchPathFinder(PathFinderInterface):
  """Base class for common Best First Search algorithms."""
  def __init__(self, road_network : nx.Graph, weight='length'):
    self.road_network = road_network
    self.weight = weight

  def heuristic_cost(self, p1_id : str, p2_id : str):
    """Returns the cartesian distance between points."""
    return NotImplementedError("Must implement heuristic_cost")

  def find_shortest_path(self, origin_id, destination_id):
    """Find the shortest path from origin_id to destination_id.

    Returns None when route not found.
    """
    settled = set()
    is_from = dict()
    g_score = defaultdict(lambda : float('inf'))
    frontier = list()
    # Initialize
    g_score[origin_id] = 0.0
    frontier.append((0.0, origin_id))
    metadata = dict()

    while len(frontier) > 0:
      unused_f_score, node_id = heapq.heappop(frontier)
      if node_id == destination_id:
        route = _reconstruct_route(origin_id, destination_id, is_from)
        metadata['settled'] = len(settled)
        return route, metadata
      if node_id in settled:
        continue
      settled.add(node_id)
      node_cost = g_score[node_id]

      for neighbor_id, edges in self.road_network[node_id].items():
        if neighbor_id in settled:
          continue

        # It is possible to have multiple arcs for same (origin, destination) pair.
        edge_cost = min([attrib[self.weight] for attrib in edges.values()])
        tentative_new_cost = node_cost + edge_cost
        if tentative_new_cost < g_score[neighbor_id]:
          g_score[neighbor_id] = tentative_new_cost
          is_from[neighbor_id] = node_id
          h_score = self.heuristic_cost(neighbor_id, destination_id)
          heapq.heappush(frontier, (tentative_new_cost + h_score, neighbor_id))

    # No route found
    metadata['settled'] = len(settled)
    return None, metadata


class DijkstraPathFinder(BestFirstSearchPathFinder):
  """Class implementing Dijkstra algorithm."""
  def __init__(self, road_network, weight='length'):
    BestFirstSearchPathFinder.__init__(self, road_network, weight)

  def get_name(self):
    return "Dijkstra Path Finder"

  def heuristic_cost(self, p1_id, p2_id):
    # Dijkstra is essentially A* without heuristic function
    return 0.0


class AStarPathFinder(BestFirstSearchPathFinder):
  """Implementation for A* algorthim."""
  def __init__(self, road_network, weight='length'):
    BestFirstSearchPathFinder.__init__(self, road_network, weight)
    # Precompute cartesian distance heuristic constants:
    #  Meters per deg lat: 2 * PI * R / 360.0
    #  Meters per deg lon: 2 * PI * r / 360.0  (r = R * cos(lat))
    #  Where R is the radius of the earth, r is R * cos(lat),
    #  and we use the median latitude only for simplicity.
    self.meter_per_deg_lat = 2 * math.pi * EARTH_RADIUS / 360.0
    lat = numpy.median(
      [n['y'] for n in self.road_network.nodes.values()])
    self.meter_per_deg_lon = (
      self.meter_per_deg_lat * math.cos(lat / 180.0 * math.pi))

  def get_name(self):
    return "A* (Euclidean Distance) Path Finder"

  def heuristic_cost(self, p1_id, p2_id):
    """Returns the cartesian distance between points."""
    p1_node = self.road_network.nodes[p1_id]
    p2_node = self.road_network.nodes[p2_id]
    p1_lat, p1_lon = p1_node['y'], p1_node['x']
    p2_lat, p2_lon = p2_node['y'], p2_node['x']
    return math.sqrt(
        math.pow((p1_lat - p2_lat) * self.meter_per_deg_lat, 2) +
        math.pow((p1_lon - p2_lon) * self.meter_per_deg_lon, 2))


def _landmark_selection(nodes : list, k) -> set:
  """Psudo random landmark selection."""
  large_prime = 999331
  small_prime = 4999
  landmark_ids = set()
  for i, node_id in enumerate(nodes):
    if (large_prime * i + small_prime) % len(nodes) < k:
      landmark_ids.add(node_id)
  return landmark_ids


def _all_costs(road_network: nx.Graph, landmark_id : str, weight : str = "length") -> dict:
  """Calculate all costs from landmark to other nodes in the network"""
  # run Dijkstra till all nodes are settled.
  settled = set()
  cost = defaultdict(lambda : float('inf'))
  frontier = list()
  cost[landmark_id] = 0.0
  frontier.append((0.0, landmark_id))

  while len(frontier) > 0:
    node_cost, node_id = heapq.heappop(frontier)
    if node_id in settled:
      continue
    settled.add(node_id)

    for neighbor_id, edges in road_network[node_id].items():
      if neighbor_id in settled:
        continue
      # Possible to have multiple arcs between same origin, destination pari.
      edge_cost = min([attrib[weight] for attrib in edges.values()])
      tentative_new_cost = node_cost + edge_cost
      if tentative_new_cost < cost[neighbor_id]:
        cost[neighbor_id] = tentative_new_cost
        heapq.heappush(frontier, (tentative_new_cost, neighbor_id))

  return cost


def _precompute_landmark_distance(
    road_network : nx.Graph, landmark_ids : set, weight : str = 'length') -> dict:
  landmark_distance = dict()
  for landmark_id in landmark_ids:
    landmark_distance[landmark_id] = _all_costs(
      road_network, landmark_id, weight=weight)
  return landmark_distance

class LandmarkPathFinder(BestFirstSearchPathFinder):
  """Implementation for A* algorthim."""
  def __init__(self, road_network, weight='length', num_landmarks=30):
    BestFirstSearchPathFinder.__init__(self, road_network, weight)
    logging.info("Start landmark precompute, %d landmark selected ...", num_landmarks)
    landmark_ids = _landmark_selection(list(road_network.nodes), num_landmarks)
    self.from_landmarks = _precompute_landmark_distance(road_network, landmark_ids, weight)
    self.to_landmarks = _precompute_landmark_distance(
      nx.reverse_view(road_network), landmark_ids, weight)

  def get_name(self):
    return "Landmark (A* using landmark heuristic) Path Finder"

  def heuristic_cost(self, p1_id, p2_id):
    """Returns the cartesian distance between points."""
    heuristic_cost = float('-inf')
    for landmark_id in self.from_landmarks.keys():
      from_landmark_cost = (
        self.from_landmarks[landmark_id][p2_id] - self.from_landmarks[landmark_id][p1_id])
      to_landmark_cost = (
        self.to_landmarks[landmark_id][p1_id] - self.to_landmarks[landmark_id][p2_id])
      tentative_heuristic_cost = max(from_landmark_cost, to_landmark_cost)
      if tentative_heuristic_cost > heuristic_cost:
        heuristic_cost = tentative_heuristic_cost
    return heuristic_cost
