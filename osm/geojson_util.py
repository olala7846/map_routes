"""
geojson_util.py
"""

from geojson import Feature, LineString
from .way import Way
from .node import Node

def to_coordinate(node: Node):
  '''Geo JSON coordinates are ordered (lon, lat)'''
  return (node.get_lon(), node.get_lat())


def way_to_geo_json(way: Way, nodes_dict: dict):
  '''Convert OSM way into GeoJSON LineString'''
  nodes = [to_coordinate(nodes_dict[nid]) for nid in way.get_node_ids()]
  properties = {
    'name': way.get_name()
  }
  line_string = LineString(nodes, properties)
  return Feature(geometry=line_string, properties=properties)
