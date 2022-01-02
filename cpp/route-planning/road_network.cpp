#include "road_network.h"

#include <iostream>
#include "osm_parser.h"

namespace hcchao
{

MapNode::MapNode(int64_t id, double lat, double lon)
  : id_(id), lat_(lat), lon_(lon) {};


// Consturctor
RoadNetwork::RoadNetwork() {};

// Destructor
RoadNetwork::~RoadNetwork() {};

bool RoadNetwork::readFromOsmFile(std::string filename) {
  OSMParser parser(filename);
  if (!parser.ParseXml(nodes)) {
    std::cout << "Error pasing file(" << filename
              << ") with error: " << parser.GetErrorMessage();
    return false;
  }
  // not implemented
  return true;
}

} // namespace hcchao

