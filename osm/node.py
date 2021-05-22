"""
node.py
"""

from xml.etree.ElementTree import Element
from .exceptions import OsmXmlParseError

OSM_XML_TAG__NODE = 'node'
OSM_XML_ATTRIB__NODE_LAT = 'lat'
OSM_XML_ATTRIB__NODE_LON = 'lon'


class Node:
  """
  A class represents an OSM Node element
  See https://wiki.openstreetmap.org/wiki/Node
  """

  def __init__(self, xml_elem: Element):
    if xml_elem.tag != OSM_XML_TAG__NODE:
      raise OsmXmlParseError(
          'Unexpected tag name {} while parsing {}'.format(xml_elem.tag, self.__class__))

    self.node_id = xml_elem.attrib['id']
    self.lat = float(xml_elem.attrib[OSM_XML_ATTRIB__NODE_LAT])
    self.lon = float(xml_elem.attrib[OSM_XML_ATTRIB__NODE_LON])

  def get_node_id(self) -> str:
    return self.node_id

  def get_lat(self) -> float:
    return self.lat

  def get_lon(self) -> float:
    return self.lon
