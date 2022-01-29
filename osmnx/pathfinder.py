"""Path finder algorithm library"""

from collections import defaultdict
import heapq
import math
import numpy


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


class DijkstraPathFinder():
  """Class implementing Dijkstra algorithm."""
  def __init__(self, road_network, weight='length'):
    self.name = "Dijkstra"
    self.road_network = road_network
    self.weigth = weight

  def find_shortest_path(self, origin_id, destination_id):
    """Find the shortest path from origin_id to destination_id.

    Returns None when route not found.
    """
    settled = set()
    frontier = list() # priority queue
    cost = defaultdict(lambda: float("inf"))
    is_from = dict()
    # Initialize origin
    cost[origin_id] = 0.0
    frontier.append((0.0, origin_id))

    while len(frontier) > 0:
      node_cost, node_id = heapq.heappop(frontier)
      if node_id == destination_id:
        return _reconstruct_route(origin_id, destination_id, is_from)
      settled.add(node_id)

      for neighbor_id, edge in self.road_network[node_id].items():
        if neighbor_id in settled:
          continue
        tentative_new_cost = node_cost + edge[0][self.weigth]
        if tentative_new_cost < cost[neighbor_id]:
          cost[neighbor_id] = tentative_new_cost
          is_from[neighbor_id] = node_id
          heapq.heappush(frontier, (tentative_new_cost, neighbor_id))
    return None


class AStarPathFinder():
  """Implementation for A* algorthim."""
  def __init__(self, road_network, weight='length'):
    self.road_network = road_network
    self.weight = weight
    # 2 * PI * R / 360.0
    self.meter_per_deg_lat = 2 * math.pi * EARTH_RADIUS / 360.0
    # use the median latitude to calculate meter per degree longitude
    lat = numpy.median(
      [n['y'] for n in self.road_network.nodes.values()])
    # 2 * PI * r / 360.0  (r = R * cos(lat))
    self.meter_per_deg_lon = (
      self.meter_per_deg_lat * math.cos(lat / 180.0 * math.pi))
    self.name = "A-Star(Cartesian distance)"

  def heuristic_cost(self, p1_id, p2_id):
    p1 = self.road_network.nodes[p1_id]
    p2 = self.road_network.nodes[p2_id]
    p1_lat, p1_lon = p1['y'], p1['x']
    p2_lat, p2_lon = p2['y'], p2['x']
    return math.sqrt(
        math.pow((p1_lat - p2_lat) * self.meter_per_deg_lat, 2) +
        math.pow((p1_lon - p2_lon) * self.meter_per_deg_lon, 2))


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

    while len(frontier) > 0:
      unused_f_score, node_id = heapq.heappop(frontier)
      if node_id == destination_id:
        return _reconstruct_route(origin_id, destination_id, is_from)
      settled.add(node_id)
      node_cost = g_score[node_id]

      for neighbor_id, edge in self.road_network[node_id].items():
        if neighbor_id in settled:
          continue
        tentative_new_cost = node_cost + edge[0][self.weight]
        if tentative_new_cost < g_score[neighbor_id]:
          g_score[neighbor_id] = tentative_new_cost
          is_from[neighbor_id] = node_id
          h_score = self.heuristic_cost(neighbor_id, destination_id)
          heapq.heappush(frontier, (tentative_new_cost + h_score, neighbor_id))
    return None

