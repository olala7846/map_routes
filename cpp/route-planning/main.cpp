#include <iostream>
#include "osm_parser.h"

int main(int argc, char* arvg[]) {
  std::cout << "Hello World from main!" << std::endl;

  hcchao::OSMParser parser("taipei_daan_map.osm");
  parser.ParseNodes();

  // Other terminations and cleanup.
  return 0;
}
