"""Path finder algorithm library"""

import heapq
import logging
import math
import networkx as nx
import osmnx as ox
import numpy
import time
from collections import defaultdict
from collections import namedtuple


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
  def __init__(self, road_network : nx.Graph, weight='length'):
    self.road_network = road_network
    self.weight = weight

  def get_name(self) -> str:
    """Returns the human readable name of the pathfinder."""
    return NotImplementedError("Must implement get_name")

  def route(self, orig, dest):
    orig_id = ox.distance.nearest_nodes(self.road_network, X=orig[1], Y=orig[0])
    dest_id = ox.distance.nearest_nodes(self.road_network, X=dest[1], Y=dest[0])
    shortest_route, unused_metadata = self.find_shortest_path(orig_id, dest_id)
    path = []
    for node_id in shortest_route:
      node_data = self.road_network.nodes[node_id]
      path.append([node_data['y'], node_data['x']])
    return path

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
        metadata['settled'] = settled
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
    metadata['settled'] = settled
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

  def route_latlng(self, route):
    """Returns an array of points(lat,lng) from route (array of node IDs)"""
    path = []
    for node_id in route:
      node_data = self.road_network.nodes[node_id]
      path.append((node_data['y'], node_data['x']))
    return path

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
  generated = 0
  total = len(landmark_ids)
  for landmark_id in landmark_ids:
    landmark_distance[landmark_id] = _all_costs(
      road_network, landmark_id, weight=weight)
    generated += 1
    logging.info('%d/%d landmark generated', generated, total)
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


class BoundingBox:
  """Represent a bounding box on the map"""
  def __init__(self, north : float, south : float, east : float, west : float):
    if not -90.0 <= south < north <= 90.0:
      raise ValueError("Invalid bbox North: {}, South: {}".format(north, south))
    if not (-180.0 <= east <= 180.0 or -180.0 <= west <= 180.0):
      raise ValueError("Invalid bbox East: {}, West: {}".format(east, west))
    self.north = north
    self.south = south
    self.east = east
    self.west = west

  def contains(self, lat : float, lng : float):
    if not -90.0 <= lat <= 90.0:
      raise ValueError("Invalid latitude {}".format(lat))
    if not (-180.0 <= self.east <= 180.0 or -180.0 <= self.west <= 180.0):
      raise ValueError("Invalid longitude".format(lng))

    if not self.south <= lat <= self.north:
      return False
    if self.west <= self.east:
      return self.west <= lng <= self.east
    else:
      return self.west <= lng and lng <= self.east


class ArcFlagPathFinder(PathFinderInterface):
  """Implement Arc Flag algorithm for a given region (bounding box)."""
  def __init__(self, road_network, region : BoundingBox, weight='length', flag_name='R'):
    self.road_network = road_network
    self.weight = weight
    self.region = region
    self.flag_name = flag_name
    precompute_start_time = time.process_time()
    self._precompute_arc_flags()
    precompute_end_time = time.process_time()
    logging.info(
      'ArcFlag precompute took %d seconds',
      precompute_end_time - precompute_start_time)

  def get_name(self) -> str:
    """Returns the human readable name of the pathfinder."""
    return "ArcFlagPathfinder"

  def _precompute_arc_flags(self):
    """Populate arc flags for the given region (bounding box)."""
    inbound_arcs = self._get_all_inbound_arcs()
    logging.info("Populating arc flags (%d inbound arcs)...", len(inbound_arcs))
    for arc_id in inbound_arcs:
      dest_id = arc_id[1]  # arc_id is a tuple of (src_id, dest_id, index)
      self._populate_arc_flag(dest_id)
    # Also mark all flags in bbox as a path to region.
    for edge_key, edge_data in self.road_network.edges.items():
      src_id, dest_id, unused_idx = edge_key
      src_node = self.road_network.nodes[src_id]
      dest_node = self.road_network.nodes[dest_id]
      if (self.region.contains(src_node['y'], src_node['x']) and
          self.region.contains(dest_node['y'], dest_node['x'])):
        edge_data[self.flag_name] = True

  def _get_all_inbound_arcs(self) -> list:
    """Returns all inbound arc IDs for the region."""
    inbound_arc_ids = list()
    for edge_key, unused_edge_data in self.road_network.edges.items():
      src_id, dest_id, unused_edge_index = edge_key
      src_node = self.road_network.nodes[src_id]
      if self.region.contains(src_node['y'], src_node['x']):
        # source node in region, can't possibly be a inbound arc.
        continue
      dest_node = self.road_network.nodes[dest_id]
      if self.region.contains(dest_node['y'], dest_node['x']):
        inbound_arc_ids.append(edge_key)
    return inbound_arc_ids

  def _populate_arc_flag(self, node_id):
    """Populate arc flags comming toward this node."""
    arcs_to_flag = self._shortest_path_arcs(node_id)
    for arc_key in arcs_to_flag:
      self.road_network.edges[arc_key][self.flag_name] = True

  def _shortest_path_arcs(self, destination_id):
    """
    Find all arcs that forms a shortest from any node to destination node
    in the road_network.
    Returns:
      A set of arc IDs (source_id, dest_id, index)
    """
    road_network = nx.reverse_view(self.road_network)
    all_arcs = set() # return value
    # Run Dijkstra
    settled = set()
    frontier = list([(0.0, destination_id, None)])
    while len(frontier) > 0:
      node_cost, node_id, arc_id = heapq.heappop(frontier)
      if node_id in settled:
        continue
      settled.add(node_id)
      if arc_id:  # Add the inbound arc of the settled node to return set
        all_arcs.add(arc_id)

      for neighbor_id, edges in road_network[node_id].items():
        if neighbor_id in settled:
          continue
        # It is possible to have multiple arcs for same (origin, destination) pair.
        inbound_arc = None
        min_edge_cost = float('inf')
        for edge_id, attrib in edges.items():
          if attrib[self.weight] < min_edge_cost:
            min_edge_cost = attrib[self.weight]
            inbound_arc = (neighbor_id ,node_id, edge_id)

        new_cost = node_cost + min_edge_cost
        heapq.heappush(frontier, (new_cost, neighbor_id, inbound_arc))
    return all_arcs

  def find_shortest_path(self, origin_id : str, destination_id : str):
    dest_node = self.road_network.nodes[destination_id]
    if not self.region.contains(dest_node['y'], dest_node['x']):
      raise ValueError('Destination (ID {}) not in region'.format(destination_id))

    settled = set()
    is_from = dict()
    cost = defaultdict(lambda : float('inf'))
    frontier = list()
    # Initialize
    cost[origin_id] = 0.0
    frontier.append((0.0, origin_id))
    metadata = dict()

    while len(frontier) > 0:
      node_cost, node_id = heapq.heappop(frontier)
      if node_id == destination_id:
        route = _reconstruct_route(origin_id, destination_id, is_from)
        metadata['settled'] = settled
        return route, metadata
      if node_id in settled:
        continue
      settled.add(node_id)

      for neighbor_id, edges in self.road_network[node_id].items():
        if neighbor_id in settled:
          continue

        # It is possible to have multiple arcs for same (origin, destination) pair.
        for attrib in edges.values():
          # Only explore arc if it is flagged.
          if self.flag_name in attrib:
            tentative_new_cost = node_cost + attrib[self.weight]
            if tentative_new_cost < cost[neighbor_id]:
              cost[neighbor_id] = tentative_new_cost
              is_from[neighbor_id] = node_id
              heapq.heappush(frontier, (tentative_new_cost, neighbor_id))

    # No route found
    metadata['settled'] = settled
    return None, metadata
