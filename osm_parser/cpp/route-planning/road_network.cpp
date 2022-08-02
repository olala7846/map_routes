#include "road_network.h"

#include <iostream>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

namespace hcchao
{

using ::rapidxml::xml_node;

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
float GetWayCost(xml_node<>* way) {
  // See max speed in United States of America for Michigan
  // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed
  static std::unordered_map<std::string, double> max_speed({
    {"motorway", 70.0},
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

    std::string tag_val(tag->first_attribute("v")->value());
    std::cout << "found way with k= " << tag_key << " v=" << tag_val << "\n";
    auto itr = max_speed.find(tag_val);
    if (itr != max_speed.end()) {
      speed = itr->second;
    }
  }
  // TODO(calculate cost): cost = distance / speed.
  return speed;
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
    float cost = GetWayCost(way);

    rapidxml::xml_node<> *nd = way->first_node("nd");
    int64_t src_id, dest_id;
    bool is_first_nd = true;
    while(nd != 0) {
      src_id = dest_id;
      dest_id = std::stoll(nd->first_attribute("ref")->value());
      if (is_first_nd) {  // skip the first node
        is_first_nd = false;
        continue;
      }
      int src_index = osmid_to_idx[src_id];
      adjacent_arcs[src_index].emplace_back(dest_id, cost);
      arc_cnt++;
      // TODO calculate cost here.



      // See max speed in United States of America for Michigan
      // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed
      // static std::unordered_map<std::string, double> max_speed({
      //   {"motorway", 70.0},
      //   {"trunk", 55.0},
      //   {"primary", 55.0},
      //   {"secondary", 45.0},
      //   {"tertiary", 35.0},
      //   {"unclassified", 55.0},
      //   {"residential", 25.0},
      //   {"living_street", 25.0},
      //   {"service", 25.0},
      // });
      // bool has_highway_tag = false;
      // bool is_one_way = false;
      // std::string highway;

      nd = nd->next_sibling("nd");
    }
    way = way->next_sibling("way");
  }
  std::cout << "Total Arcs found:" << arc_cnt << std::endl;

  return true;
}

} // namespace hcchao

