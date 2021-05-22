"""
plot_osm.py
"""

from osm.parser import parse_xml

OSM_FILE = 'taipei.osm'


def main():
  nodes, ways = parse_xml(OSM_FILE)
  for way in ways.values():
    if way.get_name():
      print(way.get_name() + ' has ' +
            str(way.get_nodes_count()) + ' nodes, type: ' + way.get_type())


if __name__ == '__main__':
  main()
