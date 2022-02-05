# Utility function
import osmnx as ox
import networkx as nx
import logging
import random
from os.path import exists
from proto.gen import road_network_pb2
from collections import defaultdict

def calc_path_cost(
    path : list, road_network : nx.Graph, weight : str ='length'):
  """Computes the total cost of a path."""
  if len(path) < 2:
    return 0.0
  total_cost = 0.0
  for n1_id, n2_id in zip(path[:-1], path[1:]):
    min_cost = float('inf')
    for attrib in road_network[n1_id][n2_id].values():
      # print('attrib: {}'.format(attrib))
      if weight in attrib:
        min_cost = attrib[weight] if attrib[weight] < min_cost else min_cost
    total_cost = total_cost + min_cost
  return total_cost


BASE_NETWORK_PATH = "./network/"

SUPPORTED_FILE_EXT = ['gpickle', 'graphml', 'pb']

def get_filename(region_name : str, extension : str = 'graphml'):
  """Normalize a region name and returns the network file name.

  Example:
    >>> get_filename('Taipei Taiwan')
    './network/taipei_taiwan.gpickle'
  """
  if extension not in SUPPORTED_FILE_EXT:
    raise ValueError('Unsupported file extension {}'.format(extension))

  normalized_name = region_name.lower().replace(',', '').replace(' ', '_')
  return BASE_NETWORK_PATH + normalized_name + '.' + extension


def read_from_file(file_path : str) -> nx.Graph:
  extension = file_path.split('.')[-1]
  if extension not in SUPPORTED_FILE_EXT:
    raise ValueError('Unsupported file extension {}'.format(extension))

  if not exists(file_path):
    raise ValueError('File not found {}'.format(file_path))

  if extension == 'gpickle':
    return nx.read_gpickle(file_path)
  elif extension == 'graphml':
    return ox.load_graphml(file_path)
  elif extension == 'pb':
    graph_pb = road_network_pb2.RoadNetwork()
    with open(file_path, 'rb') as f:
      graph_pb.ParseFromString(f.read())
    return proto_to_network(graph_pb)


def load_road_network(region_name : str, extension : str = 'pb') -> nx.Graph:
  """Loads a road network from file

  Example usage:
    tp = utils.load_road_network('Taipei Taiwan')
    tw = utils.load_road_network('taiwan')
  """
  filename = get_filename(region_name, extension=extension)
  road_network = read_from_file(filename)
  logging.info(
    'Road network %s loaded\n # nodes: %d, # arcs: %d',
    region_name, len(road_network.nodes), len(road_network.edges))
  return road_network


def sample_pairs(road_network : nx.Graph, sample_cnt : int = 100) -> list:
  orig_dest_pairs = []
  num_nodes = len(road_network.nodes)
  for unused_index in range(sample_cnt):
    orig_id = list(road_network.nodes.keys())[random.randint(0, num_nodes-1)]
    dest_id = list(road_network.nodes.keys())[random.randint(0, num_nodes-1)]
    orig_dest_pairs.append((orig_id, dest_id))
  return orig_dest_pairs


def network_to_proto(road_network : nx.Graph) -> road_network_pb2.RoadNetwork:
  """Converts a OSMnx road network to a RoadNetwork proto.

  This converter removes all unnecessary fields parsed from OSM to make
  the road network as compact as possible. If more fields is needed (e.g.
  arc geometry), this function and the proto file has to be updated accordingly.

  NetworkX Graph:
    https://networkx.org/documentation/stable/reference/classes/index.html
  """
  road_network_pb = road_network_pb2.RoadNetwork()
  for osmid, data in road_network.nodes.items():
    node_pb = road_network_pb.nodes.add()
    node_pb.osmid = osmid
    node_pb.latitude = data['y']
    node_pb.longitude = data['x']

  for edge_key, edge_data in road_network.edges.items():
    src_id, dest_id, unused_edge_index = edge_key
    arc_pb = road_network_pb.arcs.add()
    arc_pb.source_id = src_id
    arc_pb.destination_id = dest_id
    arc_pb.length = edge_data['length']
  return road_network_pb


def proto_to_network(road_network_pb: road_network_pb2.RoadNetwork) -> nx.Graph:
  """Convert road_network_pb2.RoadNetwork to OSMnx road network.
  Example:
    with open('./data/taipei.pb') as f:
      road_network_pb = road_network_pb2.RoadNetwork()
      road_network_pb.ParseFromString(f.read())
      road_network = utils.proto_to_network(road_network_pb)
  """
  graph = nx.MultiDiGraph()
  for node in road_network_pb.nodes:
    graph.add_node(node.osmid, y=node.latitude, x=node.longitude)

  arc_count = defaultdict(lambda : 0)
  for arc in road_network_pb.arcs:
    num_arcs_between = arc_count[(arc.source_id, arc.destination_id)]
    arc_count[(arc.source_id, arc.destination_id)] += 1
    graph.add_edge(
      arc.source_id, arc.destination_id, num_arcs_between, length=arc.length)
  return graph
