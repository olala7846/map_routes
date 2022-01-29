#include "osm_xml_utils.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>

#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>

namespace hcchao {
namespace utils {


using namespace xercesc;

// Wrapper function for extracting tag name as string, and more importantly
// releasing memory resources correctly.
// https://stackoverflow.com/questions/9826518/purpose-of-xmlstringtranscode
std::string toString(const XMLCh* xmlch_str) {
  char* temp = XMLString::transcode(xmlch_str);
  std::string std_str(temp);
  XMLString::release(&temp);
  return std_str;
}

std::string getElementName(DOMElement* elem) {
  return toString(elem->getTagName());
}

std::unordered_map<std::string, std::string> getWayTags(DOMElement* elem) {
  static const XMLCh kOsmTag[] = {chLatin_t, chLatin_a, chLatin_g, chNull};
  static const XMLCh kOsmTagKey[] = {chLatin_k, chNull};
  static const XMLCh kOsmTagVal[] = {chLatin_v, chNull};

  std::unordered_map<std::string, std::string> ret;
  DOMNodeList* tags = elem->getElementsByTagName(kOsmTag);
  for (int i = 0; i < tags->getLength(); i++) {
    DOMElement* tag = (DOMElement* )tags->item(i);
    const XMLCh* xmlch_key = tag->getAttribute(kOsmTagKey);
    const XMLCh* xmlch_val = tag->getAttribute(kOsmTagVal);
    std::string key = toString(xmlch_key);
    std::string val = toString(xmlch_val);
    ret.insert({key, val});
  }
  return ret;
}

// Get all nodes in a way and put the into a vector
std::vector<int64_t> getWayNodes(xercesc::DOMElement* elem) {
  static const XMLCh kOsmNd[] = {chLatin_n, chLatin_d, chNull};
  static const XMLCh kOsmNdRef[] = {chLatin_r, chLatin_e, chLatin_f, chNull};
  std::vector<int64_t> ret;
  DOMNodeList* nodes = elem->getElementsByTagName(kOsmNd);
  for (int i = 0; i < nodes->getLength(); i++) {
    DOMElement* tag = (DOMElement* )nodes->item(i);
    const XMLCh* ref_val = tag->getAttribute(kOsmNdRef);
    std::string node_id = toString(ref_val);
    ret.push_back(strtoll(node_id.c_str(), nullptr, 0));
  }
  return ret;
}

HighwayType toEnum(std::string type_str) {
  if (type_str == "motorway")
    return HighwayType::kMotorway;
  if (type_str == "trunk")
    return HighwayType::kTrunk;
  if (type_str == "primary")
    return HighwayType::kPrimary;
  if (type_str == "secondary")
    return HighwayType::kSecondary;
  if (type_str == "tertiary")
    return HighwayType::kTertiary;
  if (type_str == "unclassified")
    return HighwayType::kUnknown;
  if (type_str == "living_street")
    return HighwayType::kLivingStreet;
  if (type_str == "service")
    return HighwayType::kService;
  std::cout << "[Error] unexpected highway type: " << type_str << std::endl;
  return HighwayType::kUnknown;
}

} // namespace utils
} // namespace hcchao
