#ifndef HCCHAO_OSM_XML_UTILS_H_
#define HCCHAO_OSM_XML_UTILS_H_

#include <string>
#include <unordered_map>

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include "road_network.h"

namespace hcchao {
namespace utils {

// Converts XMLCh* string to std::string
std::string toString(const XMLCh* xmlch_str);

// Get name of an XML element.
std::string getElementName(xercesc::DOMElement* elem);

// Get all tags in a way and put all the key-value paris into a map
std::unordered_map<std::string, std::string> getWayTags(xercesc::DOMElement* elem);

// Gets all nodes (IDs) of an OSM way and put them in a vector.
std::vector<int64_t> getWayNodes(xercesc::DOMElement* elem);

// Converts std::string into HighwayType enum class
HighwayType toEnum(std::string type_str);


} // namespace utils
} // namespace hcchao

#endif   //HCCHAO_OSM_XML_UTILS_H_
