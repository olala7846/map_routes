#include "road_network.h"

#include <iostream>
#include <cmath>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

namespace hcchao
{

using ::rapidxml::xml_node;

// The radius of earth is 6.378 km
const static float earth_radius_meter = 6.378e6;
// 1 mph equals 0.44704 meter per second.
const static float mph_to_mps = 0.44704;

MapNode::MapNode(int64_t osmid, double lat, double lon)
  : osmid_(osmid), lat_(lat), lon_(lon) {};

MapArc::MapArc(int64_t destination_id, float cost)
  : destination_id_(destination_id), cost_(cost) {};


// Consturctor
RoadNetwork::RoadNetwork() {};

// Destructor
RoadNetwork::~RoadNetwork() {};

// Calculates the XML way cost.
// A way node looks like the following:
//   <way id=12345>
//    <nd ref="1001"/>
//    <nd ref="1002"/>
//    <nd ref="1003"/>
//    <tag k="highway" v="residential">
//   </way>
// we iterate through all tag and find the first highway.
float GetWaySpeedLimit(xml_node<>* way) {
  // See max speed in United States of America for Michigan
  // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed#United_States_of_America
  // The speed is is for U.S.A only (mph).
  static std::unordered_map<std::string, double> max_speed({
    {"motorway", 65.0},
    {"trunk", 55.0},
    {"primary", 55.0},
    {"secondary", 45.0},
    {"tertiary", 35.0},
    {"unclassified", 55.0},
    {"residential", 25.0},
    {"living_street", 25.0},
    {"service", 25.0},
  });

  float speed = 25.0;  // default to speed of service road
  xml_node<>* tag = way->first_node("tag");
  while(tag != 0) {
    std::string tag_key(tag->first_attribute("k")->value());
    if (tag_key != "highway") {
      tag = tag->next_sibling("tag");
      continue;
    }

    // update counter dictionary
    std::string tag_val(tag->first_attribute("v")->value());
    auto itr = max_speed.find(tag_val);
    if (itr != max_speed.end()) {
      speed = itr->second;
    }
    break;
  }
  // TODO(calculate cost): cost = distance / speed.
  return speed;
}

// Converts degree to radian.
float deg2rad(float degree) {
  return degree / 180.0 * 3.141592653589793238;
}

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

  // resize the adjacency list and initilaize with default constructor.
  adjacent_arcs.resize(nodes.size());
  std::unordered_map<int64_t, int> osmid_to_idx;
  for (int i = 0; i < nodes.size(); i++) {
    int64_t osmid = nodes[i].osmid();
    osmid_to_idx.insert({osmid, i});
  }

  // Iterate over all ways
  rapidxml::xml_node<> *way = osm->first_node("way");
  int arc_cnt = 0;
  while(way != 0) {
    // A way node looks like the following:
    //   <way id=12345>
    //    <nd ref="1001"/>
    //    <nd ref="1002"/>
    //    <nd ref="1003"/>
    //    <tag k="highway" v="residential">
    //   </way>
    // we use the highway type to infer speed.
    // we iterate through all segments of the way and construct the way.
    float speed_mph = GetWaySpeedLimit(way);

    rapidxml::xml_node<> *nd = way->first_node("nd");
    int64_t src_osmid, dest_osmid;
    bool is_first_nd = true;
    while(nd != 0) {
      src_osmid = dest_osmid;
      dest_osmid = std::stoll(nd->first_attribute("ref")->value());
      if (is_first_nd) {  // skip the first node
        is_first_nd = false;
        continue;
      }
      int src_node_index = osmid_to_idx[src_osmid];

      // Calculate the (approximate) great-circle distance
      // since most of the arcs are close by (< 1km), calculating distance
      // using the harversine formula is time consuming and can have large
      // floating point rounding errors. Instead here I simply calculate
      // the cord length instead.
      // https://en.wikipedia.org/wiki/Great-circle_distance
      const MapNode& src_node = nodes[src_node_index];
      const MapNode& dest_node = nodes[osmid_to_idx[dest_osmid]];
      float phi1 = deg2rad(src_node.lat());
      float lam1 = deg2rad(src_node.lon());
      float phi2 = deg2rad(dest_node.lat());
      float lam2 = deg2rad(dest_node.lon());
      float dx = (std::cos(phi2) * std::cos(lam2)
                  - std::cos(phi1) * std::cos(lam1));
      float dy = (std::cos(phi2) * std::sin(lam2)
                  - std::cos(phi1) * std::sin(lam1));
      float dz = std::sin(phi2) - std::sin(phi1);
      float chord = std::sqrt(dx*dx + dy*dy + dz*dz) * earth_radius_meter;
      float cost_seconds = chord / (speed_mph * mph_to_mps);
      adjacent_arcs[src_node_index].emplace_back(dest_osmid, cost_seconds);
      arc_cnt++;

      nd = nd->next_sibling("nd");
    }
    way = way->next_sibling("way");
  }
  std::cout << "Total Arcs found:" << arc_cnt << std::endl;

  return true;
}

} // namespace hcchao

