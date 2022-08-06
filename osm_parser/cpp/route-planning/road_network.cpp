#include "road_network.h"

#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <utility>
#include <map>

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
RoadNetwork::RoadNetwork() {
  reduced_ = false;
};

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

  // TODO: speed can be read directly from tags keyed by "maxspeed",
  // "maxspeed:forward", "maxspeed:backward".

  float speed = 25.0;  // default to speed of service road
  bool has_speed = false;
  xml_node<>* tag = way->first_node("tag");
  std::unique_ptr<std::string> highway_ptr;
  while(tag != 0) {
    std::string tag_key(tag->first_attribute("k")->value());
    if (tag_key == "highway") {
      std::string highway(tag->first_attribute("v")->value());
      highway_ptr = std::make_unique<std::string>(highway);
      break;
    }
    tag = tag->next_sibling("tag");
  }

  // must check before dereference, otherwise segfault
  if (highway_ptr != nullptr) {
    // Infer speed from road type
    auto itr = max_speed.find(*highway_ptr);
    if (itr != max_speed.end()) {
      speed = itr->second;
    }
  }

  return speed;
}

// Determines whether a way is one way only. Returns true if one way.
float IsWayOneWay(xml_node<>* way) {
  xml_node<>* tag = way->first_node("tag");
  while(tag != 0) {
    std::string tag_key(tag->first_attribute("k")->value());
    if (tag_key == "oneway") {
      std::string tag_val(tag->first_attribute("v")->value());
      return (tag_val == "yes")? true : false;
    }
    tag = tag->next_sibling("tag");
  }
  // unspecified.
  return false;
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
  for (int i = 0; i < nodes.size(); i++) {
    int64_t osmid = nodes[i].osmid();
    osmid_to_index_.insert({osmid, i});
  }

  // Iterate over all ways
  rapidxml::xml_node<> *way = osm->first_node("way");
  int arc_cnt = 0;
  int one_way_cnt = 0;
  int two_way_cnt = 0;
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
    bool one_way = IsWayOneWay(way);
    if (one_way) {
      ++one_way_cnt;
    } else {
      ++two_way_cnt;
    }

    rapidxml::xml_node<> *nd = way->first_node("nd");
    // initialize first node
    int64_t src_osmid, dest_osmid;
    if (nd == 0) {
      std::cerr << "Can't find nd in way.\n";
      break;
    }
    src_osmid = std::stoll(nd->first_attribute("ref")->value());
    nd = nd->next_sibling("nd");

    while(nd != 0) {
      dest_osmid = std::stoll(nd->first_attribute("ref")->value());
      int src_node_index = osmid_to_index_[src_osmid];
      int dest_node_index = osmid_to_index_[dest_osmid];

      // Calculate the (approximate) great-circle distance
      // since most of the arcs are close by (< 1km), calculating distance
      // using the harversine formula is time consuming and can have large
      // floating point rounding errors. Instead here I simply calculate
      // the cord length instead.
      // https://en.wikipedia.org/wiki/Great-circle_distance
      const MapNode& src_node = nodes[src_node_index];
      const MapNode& dest_node = nodes[dest_node_index];
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

      // Add new node to adjacency arcs.
      adjacent_arcs[src_node_index].emplace_back(dest_osmid, cost_seconds);
      arc_cnt++;
      if (!one_way) {
        adjacent_arcs[dest_node_index].emplace_back(src_osmid, cost_seconds);
        arc_cnt++;
      }

      src_osmid = dest_osmid;
      nd = nd->next_sibling("nd");
    }
    way = way->next_sibling("way");
  }
  std::cout << "one_way: " << one_way_cnt << " two_way: " << two_way_cnt << std::endl;
  std::cout << "Total Arcs found:" << arc_cnt << std::endl;

  // Print statistics about this road network.
  std::map<int, int> counter;
  for (const auto& arc_list: adjacent_arcs) {
    int num_out_going_arcs = arc_list.size();
    if (counter.find(num_out_going_arcs) != counter.end()) {
      counter[num_out_going_arcs] += 1;
    } else {
      counter.insert({num_out_going_arcs, 1});
    }
  }

  for (auto [num_outgoing_arcs, count] : counter) {
    std::cout << "node with " << num_outgoing_arcs << " out going arcs: ";
    std::cout << count << "\n";
  }

  return true;
}

// TODO: replace error status with exception.
// Returns false if error is encountered.
void RoadNetwork::reduceToLargestConnectedComponent() {
  if (reduced_) {
    std::cout << "Already reduced to largest connected component.";
    return;  // already reduced.
  }

  std::unordered_set<int64_t> unvisited_nodes;
  for (const auto& node : nodes) {
    unvisited_nodes.insert(node.osmid());
  }
  auto next_node = unvisited_nodes.begin();
  std::vector<std::unordered_set<int64_t>> components;
  findConnectedComponents(unvisited_nodes, components);

  std::cout << "total number of components: " << components.size() << "\n";

  std::vector<int> component_size;
  for (const auto& component : components) {
    component_size.push_back(component.size());
  }
  std::sort(component_size.begin(), component_size.end());
  std::cout << "largest component is of sizes:\n";
  for (int i = 0; i < 10; i++) {
    std::cout << i+1 << ": " << component_size[component_size.size() - 1 -i] << "\n";
  }

  // TODO: Reduce road network.
  // current node network.
  throw std::logic_error{"reduceToLargestConnectedComponent not implemented yet"};

  reduced_ = true;
  return;
}

void RoadNetwork::findConnectedComponents(
    std::unordered_set<int64_t>& unexplored,
    std::vector<std::unordered_set<int64_t>>& components) {
  // Here we assumes the road network is a strongly connected directed
  // graph. So which ever node is not strongly (bi-directional) connected
  // to the connected component is considered thier own component.
  while (!unexplored.empty()) {
    int64_t start_node_id = *unexplored.begin();
    std::unordered_set<int64_t> connected_component;

    // A queue of nodes to be explored.
    std::list<int64_t> frontier {start_node_id};
    while (!frontier.empty()) {
      int64_t src_node_osmid = frontier.front();  // currend not osmid.
      frontier.pop_front();

      auto search = unexplored.find(src_node_osmid);
      if (search == unexplored.end()) {
        continue;  // skip if already explored.
      }
      unexplored.erase(search);
      connected_component.insert(src_node_osmid);

      auto outbound_arcs = adjacent_arcs[osmid_to_index_[src_node_osmid]];
      for (const auto& arc : outbound_arcs) {
        int64_t dest_node_osmid = arc.destination_id();
        frontier.push_back(dest_node_osmid);
      }
    }

    // no copy
    components.push_back(std::move(connected_component));
  }
}

} // namespace hcchao

