#include <iostream>
#include "osm_parser.h"

int main(int argc, char* arvg[]) {
  std::cout << "Hello World from main!" << std::endl;

  hcchao::RoadNetwork road_network;
  // road_network.readFromOsmFile("taipei_daan_map.osm");
  road_network.readFromOsmFile("lafeyette.osm");

  // Other terminations and cleanup.
  return 0;
}
