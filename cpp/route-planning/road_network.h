#ifndef HCCHAO_ROAD_NETWORK_H_
#define HCCHAO_ROAD_NETWORK_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

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

// Represents an arc on the map (directed connection between nodes).
class MapArc {
public:
  explicit MapArc(int64_t source_id, int64_t dest_id, HighwayType type);
  ~MapArc();

private:
  int64_t source_id_;
  int64_t destination_id_;
  HighwayType type_;
  double cost_;
  // TODO(hcchao@): road type and cost
};

// Represents a node on the map
class MapNode {
public:
  explicit MapNode(int64_t id, double lat, double lon);
  ~MapNode() = default;

  void AddArc(int64_t dest_node_id, HighwayType highway_type);

private:
  int64_t id_;
  double lat_;  // [-90.0, 90.0]
  double lon_;  // [-180.0, 180.0]
  std::vector<MapArc> outgoing_arcs_;
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
