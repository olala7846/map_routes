#include <iostream>
#include "road_network.h"

int main(int argc, char* arvg[]) {
  std::cout << "Hello World from main!" << std::endl;

  hcchao::RoadNetwork road_network;
  // road_network.readFromOsmFile("taipei_daan_map.osm");
  std::string osmfile = "lafeyette.osm";
  road_network.readFromOsmFile(osmfile);
  road_network.reduceToLargestConnectedComponent();

  // Other terminations and cleanup.
  return 0;
}
