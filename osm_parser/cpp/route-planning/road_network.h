// Author: olala7846@gmail.com
//
// Inspired by the University of Freiburg, efficient routing class design suggestion.

#ifndef HCCHAO_ROAD_NETWORK_H_
#define HCCHAO_ROAD_NETWORK_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <list>

namespace hcchao
{

enum class HighwayType {
  kUnknown,
  kMotorway,
  kTrunk,
  kPrimary,
  kSecondary,
  kTertiary,
  kUnclassified,
  kResidential,
  kLivingStreet,
  kService
};

// Class representing an OSM Node. See https://wiki.openstreetmap.org/wiki/Node.
class MapNode {
 public:
  explicit MapNode(int64_t osmid_, double lat, double lon);
  ~MapNode() = default;

  int64_t osmid() { return osmid_; }

 private:
  // ID is a 64-bit integer number. See https://wiki.openstreetmap.org/wiki/Node#Structure
  int64_t osmid_;

  // OSM define lat to be decimal numer [-90.0, 90.0] with 7 decimal places
  // OSM define lon to be decimal numer [-180.0, 180.0] with 7 decimal places
  // float is enough for such precision. For more details, see
  // https://wiki.openstreetmap.org/wiki/Precision_of_coordinates
  float lat_;  // [-90.0, 90.0]
  float lon_;  // [-180.0, 180.0]
};

// Represents an arc (edge) on the map (directed connection between nodes).
class MapArc {
 public:
  explicit MapArc(int64_t destination_id);
  ~MapArc() = default;

  // OSM ID of the arc destination node.
  int64_t destination_id() { return destination_id_; }

 private:
  int64_t destination_id_;
  float cost_;
  // TODO(hcchao@): road type and cost
};

// A directed graph parsed from OSM file.
// see readFromOsmFile method.
class RoadNetwork {
 public:
  RoadNetwork();
  ~RoadNetwork();

  // Read road network from OSM file.
  // Returns true if read operation was successful.
  bool readFromOsmFile(const std::string& filename);

  // All the nodes in the graph. The order of this vector will always be the same as
  // the adjacent_arcs below.
  std::vector<MapNode> nodes;

  // An adjacency list to store the road network.
  //
  // For sparse graph (e.g. road network), an adjacency list is more space-efficient
  // than adjacency matrix.
  //
  // Here I choose to go with the implementation approach suggested by Cormen et al.
  // see https://en.wikipedia.org/wiki/Adjacency_list#Implementation_details
  //
  // The graph is represented by a vector indexed by vertex number, in which vector
  // cell for each vertex points to a singly linked list of the neighboring vertices.
  // only one of the two endpoints of the edge is being stored (see class MapArc).
  // Here we only store the destination node in the MapArc.
  // For example:
  // MapArc arc = adjacentArcs[3][4];
  // int64_t src_osmid = nodes[3].osmid();
  // int64_t dest_osmid = adjacentArcs[3][4].dest_id();
  std::vector<std::list<MapArc> > adjacent_arcs;
};

} // namespace hcchao


#endif   // HCCHAO_ROAD_NETWORK_H_
