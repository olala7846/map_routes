# Utility function
import osmnx as ox
import networkx as nx
import logging
import random
from os.path import exists

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
    #total_cost += road_network.edges[n1_id, n2_id, 0][weight]
  return total_cost


BASE_NETWORK_PATH = "./network/"

SUPPORTED_FILE_EXT = ['gpickle', '.graphml']

def get_filename(region_name : str, extension : str = 'gpickle'):
  if extension not in SUPPORTED_FILE_EXT:
    raise ValueError('Unsupported file extension {}'.format(extension))

  normalized_name = region_name.lower().replace(',', '').replace(' ', '_')
  return BASE_NETWORK_PATH + normalized_name + '.' + extension


def read_from_file(file_path : str):
  extension = file_path.split('.')[-1]
  if extension not in SUPPORTED_FILE_EXT:
    raise ValueError('Unsupported file extension {}'.format(extension))

  if not exists(file_path):
    raise ValueError('File not found {}'.format(file_path))

  if extension == 'gpickle':
    return nx.read_gpickle(file_path)
  elif extension == 'graphml':
    return ox.load_graphml(file_path)


def load_road_network(region_name : str):
  filename = get_filename(region_name)
  return read_from_file(filename)



def sample_pairs(road_network : nx.Graph, sample_cnt : int = 100) -> list:
  orig_dest_pairs = []
  num_nodes = len(road_network.nodes)
  for unused_index in range(sample_cnt):
    orig_id = list(road_network.nodes.keys())[random.randint(0, num_nodes-1)]
    dest_id = list(road_network.nodes.keys())[random.randint(0, num_nodes-1)]
    orig_dest_pairs.append((orig_id, dest_id))
  return orig_dest_pairs
