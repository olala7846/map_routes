# Routing Algorithms
from collections import defaultdict
import heapq
import osmnx as ox
import logging
import math


EARTH_RADIUS=6371009
# estimated one degree lat lon to meter around taipei city
ONE_DEG_LAT_TO_METER = 111195.083724
ONE_DEG_LON_TO_METER = 100690.272542

def _construct_route(came_from, orig_id, dest_id):
  route = []
  node_id = dest_id
  while node_id != orig_id:
    route.insert(0, node_id)
    node_id = came_from[node_id]
  route.insert(0, orig_id)
  return route


def dijkstra_shortest_path(graph, orig_id, dest_id, weight="length"):
  explored = set()
  came_from = dict()
  min_cost = defaultdict(lambda : float('inf'))
  frontier = [(0.0, orig_id)]  # (cost, id)
  cnt = 0
  while len(frontier) > 0:
    cnt = cnt + 1
    curr_cost, curr_node_id = heapq.heappop(frontier)
    if curr_node_id == dest_id:
      logging.info('Dijkstra number of nodes explored: %d, iter: %d', len(explored), cnt)
      return _construct_route(came_from, orig_id, dest_id)
    explored.add(curr_node_id)
    for next_node_id, edge in graph[curr_node_id].items():
      if next_node_id in explored:
        continue
      new_cost = curr_cost + edge[0][weight]
      if min_cost[next_node_id] > new_cost:
        min_cost[next_node_id] = new_cost
        came_from[next_node_id] = curr_node_id
        heapq.heappush(frontier, (new_cost, next_node_id))
  return None


def _distance(node_id, dest_id, graph):
  """Return great circle distance between two coordinates (in meters).

  Taipei City, Taiwan is 25.105497, and the longitude is 121.597366
  EARTH_RADIUS=6371009
  which means
    * one degree latitude == 111195.083724 meters
    * one degree longidue == 100690.272542 meters

  Note: ox.distance.great_circle_vec() calculates the precise
    great circle distnace but it is extremely slow. This could
    make A* search slower then Dijkstra, hence we only use the
    estimated euclidience distance near Taipei Taiwan lat/lon
  """
  curr_node = graph.nodes[node_id]
  dest_node = graph.nodes[dest_id]
  curr_node_lat, curr_node_lon = curr_node['y'], curr_node['x']
  dest_node_lat, dest_node_lon = dest_node['y'], dest_node['x']

  return math.sqrt(
      math.pow((curr_node_lat - dest_node_lat) * ONE_DEG_LAT_TO_METER, 2) +
      math.pow((curr_node_lon - dest_node_lon) * ONE_DEG_LON_TO_METER, 2))


def a_star_shortest_path(
  graph, orig_id, dest_id, weight="length"):
  explored = set()
  came_from = dict()
  g_score = defaultdict(lambda : float('inf'))
  g_score[orig_id] = 0.0
  frontier = [(0.0 + _distance(orig_id, dest_id, graph), orig_id)]
  cnt = 0
  while len(frontier) > 0:
    cnt = cnt + 1
    unused_fscore, curr_node_id = heapq.heappop(frontier)

    if curr_node_id == dest_id:
      logging.info('A* number of nodes explored: %d, iter: %d', len(explored), cnt)
      return _construct_route(came_from, orig_id, dest_id)
    explored.add(curr_node_id)

    for next_node_id, edge in graph[curr_node_id].items():
      if next_node_id in explored:
        continue
      tentative_g_score = g_score[curr_node_id] + edge[0][weight]
      if tentative_g_score < g_score[next_node_id]:
        g_score[next_node_id] = tentative_g_score
        came_from[next_node_id] = curr_node_id
        heuristic_cost = _distance(next_node_id, dest_id, graph)
        f_score = tentative_g_score + heuristic_cost
        # Use -len(frontier) as tie braker so the queue is LIFO
        tie_breaker = -len(frontier)
        new_elem = ((f_score, tie_breaker), next_node_id)
        heapq.heappush(frontier, new_elem)
  return None
