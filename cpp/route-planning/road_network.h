#ifndef HCCHAO_ROAD_NETWORK_H_
#define HCCHAO_ROAD_NETWORK_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace hcchao
{

// Represents a node on the map
class MapNode {
public:
  explicit MapNode(int64_t id, double lat, double lon);
  ~MapNode() = default;

  int64_t id_;
  double lat_;  // [-90.0, 90.0]
  double lon_;  // [-180.0, 180.0]
  // Adjancency list, representing a directed arc from this node to neighbour.
  std::vector<int64_t> neighbours;
};

// A directed graph parsed from OSM file.
// see readFromOsmFile method.
class RoadNetwork {
public:
  RoadNetwork();
  ~RoadNetwork();
  // Read road network from OSM file.
  // Returns true if read operation was successful.
  bool readFromOsmFile(std::string filename);
  std::unordered_map<int64_t, MapNode> nodes;
};

} // namespace hcchao


#endif   // HCCHAO_ROAD_NETWORK_H_
