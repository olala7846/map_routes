import xml.etree.ElementTree as ET

from collections import defaultdict

OSM_FILE = 'taipei.osm'

def main():
  tree = ET.parse(OSM_FILE)

  counter = defaultdict(int)

  for elem in tree.getroot():
    counter[elem.tag] += 1

  print(counter)


if __name__ == '__main__':
  main()
