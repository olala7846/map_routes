# Routing Algorithms
from collections import defaultdict
import heapq
import osmnx as ox

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
      new_cost = curr_cost + edge[0][weight]
      if min_cost[next_node_id] > new_cost:
        min_cost[next_node_id] = new_cost
        is_from[next_node_id] = curr_node_id
        heapq.heappush(frontier, (new_cost, next_node_id))
  return None


def _distance(node_id, dest_id, graph):
  "Return great circle distance between two coordinates (in meters)"
  curr_node = graph.nodes[node_id]
  dest_node = graph.nodes[dest_id]
  curr_node_lat, curr_node_lon = curr_node['y'], curr_node['x']
  dest_node_lat, dest_node_lon = dest_node['y'], dest_node['x']
  return ox.distance.great_circle_vec(
    curr_node_lat, curr_node_lon, dest_node_lat, dest_node_lon)


def a_star_shortest_path(graph, orig_id, dest_id, weight="length"):
  explored = set()
  is_from = dict()
  min_cost = defaultdict(lambda : float('inf'))
  min_cost[orig_id] = 0.0
  frontier = [(0.0 + _distance(orig_id, dest_id, graph), orig_id)]
  while len(frontier) > 0:
    unused_cost, curr_node_id = heapq.heappop(frontier)
    if curr_node_id == dest_id:
      return _construct_route(is_from, orig_id, dest_id)
    explored.add(curr_node_id)
    for next_node_id, edge in graph[curr_node_id].items():
      if next_node_id in explored:
        continue
      new_cost = min_cost[curr_node_id] + edge[0][weight]
      if min_cost[next_node_id] > new_cost:
        min_cost[next_node_id] = new_cost
        is_from[next_node_id] = curr_node_id
        heuristic_cost = _distance(next_node_id, dest_id, graph)
        heapq.heappush(frontier, (new_cost + heuristic_cost, next_node_id))
  return None
