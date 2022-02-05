"""
Load a road road network file (Graphml format) and serialize as proto file.
This program loads the original OSMnx road network (e.g. ./network/taipei_taiwan.graphml)
and convert it into a serialized proto format, and generate (e.g. ./network/taipei_taiwan.pb)
So it is smaller in size (12.5% original size) and can be loaded into pathfinder (20%) faster.

Usage:
  `python network_to_proto.py --region="Taipei, Taiwan"`

"""

import logging
import time
from absl import app
from absl import flags
from proto.gen import road_network_pb2
import utils

FLAGS = flags.FLAGS
flags.DEFINE_string(
  'region', 'Taipei Taiwan', 'Region name of road network to use')


def main(argv):
  t_start = time.process_time()
  road_network = utils.load_road_network(FLAGS.region, extension='graphml')
  t_end = time.process_time()
  print('Time to load network:', t_end - t_start)
  print('Number of nodes:', len(road_network.nodes))
  print('Number of edges:', len(road_network.edges))

  print("A node looks like:", road_network.nodes[7791704127])
  print("An edge looks like:", road_network.edges[(9402576490, 2612435152, 0)])
  road_network_pb = utils.network_to_proto(road_network)
  pb_filename = utils.get_filename(FLAGS.region, extension='pb')
  with open(pb_filename, 'wb') as f:
    f.write(road_network_pb.SerializeToString())
    print("Serailzied as ", pb_filename)

  # Read from pb file
  road_network_pb = road_network_pb2.RoadNetwork()
  t_start = time.process_time()
  with open(pb_filename, 'rb') as f:
    road_network_pb.ParseFromString(f.read())
    print("Read road network from ", pb_filename)
  simplified_road_network = utils.proto_to_network(road_network_pb)
  t_end = time.process_time()
  print('Time to load network:', t_end - t_start)
  print('Number of nodes:', len(simplified_road_network.nodes))
  print('Number of edges:', len(simplified_road_network.edges))
  print("A node looks like:", simplified_road_network.nodes[7791704127])
  print("An edge looks like:", simplified_road_network.edges[(9402576490, 2612435152, 0)])






if __name__ == '__main__':
  app.run(main)
