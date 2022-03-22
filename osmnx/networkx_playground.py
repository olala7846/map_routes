"""
Test nextworkx methods

Usage:
  python test_networkx.py
"""

import logging
from absl import app
from absl import flags

import networkx as nx

FLAGS = flags.FLAGS


def main(argv):
  # Test MultiDigraph
  MG = nx.MultiDiGraph()
  MG.add_nodes_from([1, 2, 3, 4, 5])
  MG.add_edge(1, 2, cost=1.0)
  MG.add_edge(1, 2, cost=1.1)
  MG.add_edge(1, 2, cost=1.2)

  print(MG.edges[1, 2, 0])
  # {'cost': 1.0}
  print(MG[1])
  # {2: {0: {'cost': 1.0}, 1: {'cost': 1.1}, 2: {'cost': 1.2}}}
  print(MG[1][2])
  # {0: {'cost': 1.0}, 1: {'cost': 1.1}, 2: {'cost': 1.2}}


if __name__ == '__main__':
  app.run(main)
