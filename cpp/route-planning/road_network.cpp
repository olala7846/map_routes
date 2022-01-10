#include "road_network.h"

#include <iostream>
#include "osm_parser.h"

namespace hcchao
{

// Constructor
MapArc::MapArc(int64_t source_id, int64_t dest_id, HighwayType type)
  : source_id_(source_id), destination_id_(dest_id), type_(type) {};

MapArc::~MapArc() {};


MapNode::MapNode(int64_t id, double lat, double lon)
  : id_(id), lat_(lat), lon_(lon) {};

void MapNode::AddArc(int64_t dest_node_id, HighwayType highway_type) {
  outgoing_arcs_.emplace_back(id_, dest_node_id, highway_type);
};

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

