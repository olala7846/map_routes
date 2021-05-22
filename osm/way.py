"""
way.py
"""

import logging

from xml.etree.ElementTree import Element
from .exceptions import OsmXmlParseError


OSM_XML_TAG__WAY = 'way'
OSM_XML_TAG__WAY_NODE = 'nd'
OSM_XML_TAG__WAY_TAG = 'tag'
OSM_XML_TAG__WAY_TAG_KEY_NAME = 'name'
OSM_XML_TAG__WAY_TAG_KEY_HIGHWAY = 'highway'


class Way:
  """
  A class represents an OSM Way element
  See https://wiki.openstreetmap.org/wiki/Way
  """

  def __init__(self, xml_elem: Element):
    if xml_elem.tag != OSM_XML_TAG__WAY:
      raise OsmXmlParseError(
          'Unexpected tag name {} while parsing {}'.format(xml_elem.tag, self.__class__))

    # attributes, congsider making them private?
    self.way_id = xml_elem.attrib['id']
    self.node_ids = list()
    self.tags = dict()

    for child in xml_elem:
      if child.tag == OSM_XML_TAG__WAY_NODE:
        self.node_ids.append(child.attrib['ref'])
      elif child.tag == OSM_XML_TAG__WAY_TAG:
        try:
          self.tags[child.attrib['k']] = child.attrib['v']
        except KeyError:
          logging.warning('Key "k" not found in tag %s', child.tag)

  def get_way_id(self):
    return self.way_id

  def get_name(self):
    try:
      return self.tags[OSM_XML_TAG__WAY_TAG_KEY_NAME]
    except KeyError:
      return None

  def get_type(self):
    try:
      return self.tags[OSM_XML_TAG__WAY_TAG_KEY_HIGHWAY]
    except KeyError:
      return "None"

  def get_nodes_count(self):
    return len(self.node_ids)
