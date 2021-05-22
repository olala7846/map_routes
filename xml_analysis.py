import xml.etree.ElementTree as ET

from collections import defaultdict

OSM_FILE = 'taipei.osm'


def main():
    tree = ET.parse(OSM_FILE)

    counter = defaultdict(int)

    for elem in tree.getroot():
        counter[elem.tag] += 1

    # Try to answer the following questions:
    # How many Nodes & Ways
    print(counter)

    # How many Ways are roads (have highyway tag)
    # How many Nodes are connected to a real road
    # Some statistic (e.g. avereage arcs per node)


if __name__ == '__main__':
    main()
