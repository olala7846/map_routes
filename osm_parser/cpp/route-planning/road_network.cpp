#include "road_network.h"

#include <iostream>
#include "osm_parser.h"

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

namespace hcchao
{

MapNode::MapNode(int64_t osmid, double lat, double lon)
  : osmid_(osmid), lat_(lat), lon_(lon) {};

MapArc::MapArc(int64_t destination_id)
  : destination_id_(destination_id) {};


// Consturctor
RoadNetwork::RoadNetwork() {};

// Destructor
RoadNetwork::~RoadNetwork() {};

// Read OSM file and construct road network and store it in class.
// The function implementation works like the following:
//  step 1, parse OSM file
//  step 2, iterate all <node> tag and populate MapNode into nodes.
//  step 3, iterate all <way> and populate arcs accordingly. And construct
//          MapArc and store in the adjacency list.
bool RoadNetwork::readFromOsmFile(const std::string& filename) {
  // step 1, parse OSM file
  // https://stackoverflow.com/questions/29474690/how-do-i-handle-rapidxml-errors
  rapidxml::file<> xml_file(filename.c_str());
  rapidxml::xml_document<> doc;
  try {
    doc.parse<0>(xml_file.data());
  }
  catch(...) {
    std::cout << "An error occured while parsing XML." << std::endl;
    return false;
  }

  // step 2, iterate all <node> tag and populate MapNode into nodes.
  rapidxml::xml_node<> *osm = doc.first_node("osm");
  rapidxml::xml_node<> *node = osm->first_node("node");

  // Parse Node
  while(node != 0) { // next_sibling returns 0 if not found
    char* p_end;
    int64_t osmid = std::stoll(node->first_attribute("id")->value());
    float lat = std::stof(node->first_attribute("lat")->value());
    float lon = std::stof(node->first_attribute("lon")->value());
    nodes.emplace_back(osmid, lat, lon);
    node = node->next_sibling("node");
  }
  std::cout << "Total number of nodes: " << nodes.size() << std::endl;

  // step 3, iterate all <way> and populate arcs accordingly. And
  // construct MapArc and store in adjacent_arcs.

  return true;
}

} // namespace hcchao

