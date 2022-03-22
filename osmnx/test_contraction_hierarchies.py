"""
CHPathFinder unit test

Usage:
  python test_contraction_hierarchies.py
  or
  python -m unittest test_contraction_hierarchies.py
"""

import unittest
import logging
from os.path import exists

from absl import app
from absl import flags
import networkx as nx
import osmnx as ox
from pathfinder import CHPathFinder

logger = logging.getLogger("contraction_hierarchies_test.py")

class TestContractionHierarchies(unittest.TestCase):

  def setUp(self):
    MG = nx.MultiDiGraph()
    MG.add_nodes_from(['N', 'S1', 'S2', 'S3', 'D1', 'D2', 'D3'])
    MG.add_edge('S1', 'N', length=5.0)
    MG.add_edge('S2', 'N', length=7.0)
    MG.add_edge('S3', 'N', length=6.0)
    MG.add_edge('N', 'D1', length=3.0)
    MG.add_edge('N', 'D2', length=4.0)
    MG.add_edge('N', 'D3', length=5.0)
    MG.add_edge('S1', 'D1', length=10.0)
    MG.add_edge('S3', 'D3', length=5.0)
    MG.add_edge('D3', 'D2', length=1.0)
    MG.add_edge('D2', 'D1', length=1.0)
    MG.add_edge('D2', 'D3', length=0.9)
    # Deleted node and edges
    MG.add_nodes_from(['S4', 'D4'], deleted=True)
    MG.add_edge('S4', 'N', length=1.0, deleted=True)
    MG.add_edge('N', 'D4', length=2.0, deleted=True)
    self.road_network = MG

  def test_add_shortcuts_S1(self):
    pathfinder = CHPathFinder(self.road_network, build_on_init=False)
    shortcuts = pathfinder.get_shortcuts_single_source(
      'N', 'S1', set(['D1', 'D2', 'D3']))
    self.assertEqual(len(shortcuts), 2)
    self.assertIn(('S1', 'D1', 8.0), shortcuts)
    self.assertIn(('S1', 'D2', 9.0), shortcuts)
    # distance ('S1', 'D3') == 9.9

  def test_add_shortcuts_S2(self):
    pathfinder = CHPathFinder(self.road_network, build_on_init=False)
    shortcuts = pathfinder.get_shortcuts_single_source(
      'N', 'S2', set(['D1', 'D2', 'D3']))
    self.assertEqual(len(shortcuts), 2)
    self.assertIn(('S2', 'D1', 10.0), shortcuts)
    self.assertIn(('S2', 'D2', 11.0), shortcuts)
    # distance ('S2', 'D3') == 11.9, no shortcut

  def test_add_shortcuts_S3(self):
    pathfinder = CHPathFinder(self.road_network, build_on_init=False)
    shortcuts = pathfinder.get_shortcuts_single_source(
      'N', 'S3', set(['D1', 'D2', 'D3']))
    # no shortcuts needs to be added
    self.assertEqual(len(shortcuts), 0)

  def test_add_shortcuts(self):
    pathfinder = CHPathFinder(self.road_network, build_on_init=False)
    pathfinder.add_shortcuts(
      'N', set(['S1', 'S2', 'S3']), set(['D1', 'D2', 'D3']))
    self.assertIn({'length': 8.0, 'is_shortcut': True, 'shortcut_for': 'N'},
      pathfinder.road_network['S1']['D1'].values())

  def test_contract_node(self):
    pathfinder = CHPathFinder(self.road_network, build_on_init=False)
    pathfinder.contract_node('N')

    # Test mark node as deleted
    self.assertTrue('N' in pathfinder.deleted_nodes)
    self.assertTrue('contraction_id' in pathfinder.road_network.nodes['N'])
    self.assertTrue(
      isinstance(pathfinder.road_network.nodes['N']['contraction_id'], int))

  def test_random_node_ordering(self):
    # Only contract number_nodes_to_contraction
    pathfinder = CHPathFinder(
      self.road_network, build_on_init=False, num_nodes_to_contract=3)
    node_order = pathfinder.random_node_ordering()
    self.assertEqual(len(node_order), 3)

    # Default contracts all the nodes.
    pathfinder2 = CHPathFinder(
      self.road_network, build_on_init=False)
    node_order2 = pathfinder2.random_node_ordering()
    self.assertEqual(len(node_order2), 9)

  def test_contraction_id(self):
    # Only contract number_nodes_to_contraction
    pathfinder = CHPathFinder(
      self.road_network, build_on_init=True, num_nodes_to_contract=5)
    contraction_ids = set()
    for node_id, node in pathfinder.road_network.nodes.items():
      if 'contraction_id'in node:
        contraction_ids.add(node['contraction_id'])
    self.assertEqual(len(contraction_ids), 5)
    self.assertEqual(contraction_ids, set([1,2,3,4,5]))

if __name__ == '__main__':
  unittest.main()
