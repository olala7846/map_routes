"""
parser.py
"""

import xml.etree.ElementTree as ET
from .node import Node, OSM_XML_TAG__NODE
from .way import Way, OSM_XML_TAG__WAY


def parse_xml(filename: str):
  """
  parse OSM XML file into Nodes and Ways

  e.g.
  nodes, ways = parse_xml('taipei.osm')
  """
  nodes = dict()
  ways = dict()
  tree = ET.parse(filename)
  for elem in tree.getroot():
    if elem.tag == OSM_XML_TAG__NODE:
      node = Node(elem)
      nodes[node.get_node_id()] = node
    elif elem.tag == OSM_XML_TAG__WAY:
      way = Way(elem)
      ways[way.get_way_id()] = way
  return nodes, ways

