#ifndef HCCHAO_OSM_XML_UTILS_H_
#define HCCHAO_OSM_XML_UTILS_H_

#include <string>
#include <unordered_map>

#include "road_network.h"

namespace hcchao {
namespace osm_utils {

// Get all tags in a way and put all the key-value paris into a map
std::unordered_map<std::string, std::string> getWayTags();

// Gets all nodes (IDs) of an OSM way and put them in a vector.
std::vector<int64_t> getWayNodes();

// Converts std::string into HighwayType enum class
HighwayType toEnum(std::string type_str);


} // namespace utils
} // namespace hcchao

#endif   //HCCHAO_OSM_XML_UTILS_H_
