import unittest
import osmnx as ox
from pathfinder import LandmarkPathFinder
from functools import reduce
from utils import calc_path_cost

class TestLandmarkPathFinder(unittest.TestCase):

  def setUp(self):
    self.road_network = ox.load_graphml('./data/taipei_taiwan.graphml')
    self.pathfinder = LandmarkPathFinder(self.road_network, num_landmarks=5)
    self.road_network_num_nodes = len(self.road_network.nodes())
    print('road network # of nodes: ' + str(self.road_network_num_nodes))

  def test_landmark(self):
    pathfinder = self.pathfinder
    self.assertEqual(len(pathfinder.from_landmarks), 5)
    self.assertEqual(len(pathfinder.to_landmarks), 5)

    for key, costs in pathfinder.from_landmarks.items():
      # Roadnetwork is not always a connected, but we assume 90% of them are connected.
      self.assertGreater(len(costs), self.road_network_num_nodes * 0.9)

    for key, costs in pathfinder.to_landmarks.items():
      # Roadnetwork is not always a connected, but we assume 90% of them are connected.
      self.assertGreater(len(costs), self.road_network_num_nodes * 0.9)

    # landmark_id = 632564801
    landmark_id = list(pathfinder.from_landmarks.keys())[0]
    road_network = pathfinder.road_network
    print('testing from_landmarks...')
    for dest_id, cost in pathfinder.from_landmarks[landmark_id].items():
      shortest_path = ox.distance.shortest_path(road_network, landmark_id, dest_id)
      path_cost = calc_path_cost(shortest_path, road_network)
      self.assertAlmostEqual(cost, path_cost, places=10)

    print('testing to_landmarks...')
    for origin_id, cost in pathfinder.to_landmarks[landmark_id].items():
      shortest_path = ox.distance.shortest_path(road_network, origin_id, landmark_id)
      path_cost = calc_path_cost(shortest_path, road_network)
      self.assertAlmostEqual(cost, path_cost, places=10)


if __name__ == '__main__':
  unittest.main()
