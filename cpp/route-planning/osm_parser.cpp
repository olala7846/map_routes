#include "osm_parser.h"

#include <iostream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <utility>

#include <xercesc/util/XMLUniDefs.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/dom/DOMAttr.hpp>

#include "osm_xml_utils.h"

namespace hcchao
{

using namespace xercesc;

OSMParser::OSMParser(std::string filename) : filename_(filename) {
  // TODO(hcchao@): find a better way to initialize XMLPlatformUtils.
  // It is usually not a good idea to run code that can throw
  // exceptions inside constructor, but I haven't find a good way
  // create an RAII that ensures Initialize() and Terminate() being called
  // once properly without doing it in the constructor.
  try {
    XMLPlatformUtils::Initialize();
    initialized_ = true;
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    std::stringstream ss;
    ss << "Error during xerces XML initialization! :\n"
       << utils::toString(toCatch.getMessage()) << "\n";
    ss >> error_msg_;
    initialized_ = false;
  }
  catch (...) {
    error_msg_ = "Unknown error during Xercesc initialization.";
    initialized_ = false;
  }
}

OSMParser::~OSMParser() {
  XMLPlatformUtils::Terminate();
}

bool OSMParser::ParseXml(std::unordered_map<int64_t, MapNode>& nodes) {
  if (!initialized_)
    return false;

  // Initialize XML parser
  std::unique_ptr<XercesDOMParser> parser(new XercesDOMParser());

  // Disabling some parser capabilities (e.g. DTD and namespace) to make
  // parsing faster. See https://wiki.openstreetmap.org/wiki/OSM_XML#Basics
  parser->setValidationScheme(XercesDOMParser::Val_Never);
  parser->setDoNamespaces(false);  // optional

  std::unique_ptr<ErrorHandler> errHandler(
      (ErrorHandler*) new HandlerBase());
  parser->setErrorHandler(errHandler.get());

  // Start parsing
  try {
    parser->parse(filename_.c_str());
  }
  catch (const XMLException& toCatch) {
    std::stringstream ss;
    ss << "Exception message is: \n"
       << utils::toString(toCatch.getMessage()) << "\n";
    ss >> error_msg_;
    return false;
  }
  catch (const DOMRangeException& toCatch) {
    std::stringstream ss;
    ss << "Exception message is: \n"
       << utils::toString(toCatch.getMessage()) << "\n";
    ss >> error_msg_;
    return false;
  }
  catch (...) {
    error_msg_ = "Unexpected Exception \n";
    return false;
  }

  DOMDocument* document = parser->getDocument();
  DOMElement* root = document->getDocumentElement();
  std::string root_tag_name = utils::getElementName(root);
  if (root_tag_name != "osm") {
    std::stringstream ss;
    ss << "Unexpected root element (" << root_tag_name << ") in XML.";
    ss >> error_msg_;
    return false;
  }

  // First, construct all nodes.
  static const XMLCh kOsmNode[] = {chLatin_n, chLatin_o, chLatin_d, chLatin_e, chNull};
  xercesc::DOMNodeList* node_list = document->getElementsByTagName(kOsmNode);

  static const XMLCh kOsmLat[] = {chLatin_l, chLatin_a, chLatin_t, chNull};
  static const XMLCh kOsmLon[] = {chLatin_l, chLatin_o, chLatin_n, chNull};
  static const XMLCh kOsmId[] = {chLatin_i, chLatin_d, chNull};

  for (XMLSize_t i = 0; i < node_list->getLength(); i++) {
    DOMElement* osm_node = (DOMElement* )node_list->item(i);
    std::string node_id = utils::toString(osm_node->getAttribute(kOsmId));
    std::string lat = utils::toString(osm_node->getAttribute(kOsmLat));
    std::string lon = utils::toString(osm_node->getAttribute(kOsmLon));
    // std::cout << "node_id: " << node_id << std::endl;
    int64_t nid = std::stoll(node_id);

    // TODO(hcchao@): find a better way to convert string to int64_t.
    MapNode new_node(nid, std::stod(lat), std::stod(lon));
    nodes.emplace(std::make_pair(nid, new_node));

    // TODO(hcchao@): skipping node tags for now, add it in the future.
    if ((i+1) % 100 == 0) {
      std::cout << "parsed " << i+1 << " nodes\n";
    }
  }
  std::cout << "total " << nodes.size() << " nodes being parsed!\n";

  // Second, populate adjacency list from ways.
  // https://wiki.openstreetmap.org/wiki/Way#Examples
  //
  // <way id="12345">
  //   <nd ref="822403"/>
  //   <nd ref="822404"/>
  //   <nd ref="822405"/>
  //     ...
  //   <tag k="highway" v="motorway"/>
  //   <tag k="name" v="Clipstone Street"/>
  //   <tag k="oneway" v="yes"/>
  // </way>

  static const XMLCh kOsmWay[] = {chLatin_w, chLatin_a, chLatin_y, chNull};
  static const XMLCh kOsmTag[] = {chLatin_t, chLatin_a, chLatin_g, chNull};
  static const XMLCh kOsmTagKey[] = {chLatin_k, chNull};
  static const XMLCh kOsmTagVal[] = {chLatin_v, chNull};
  static const XMLCh kOsmTagKeyHighway[] = {
    chLatin_h, chLatin_i, chLatin_g, chLatin_h,
    chLatin_w, chLatin_a, chLatin_y, chNull
  };
  xercesc::DOMNodeList* way_list = document->getElementsByTagName(kOsmWay);

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
  bool has_highway_tag = false;
  bool is_one_way = false;
  std::string highway;

  for (XMLSize_t i = 0; i < way_list->getLength(); i++) {
    DOMElement* way_node = (DOMElement *)way_list->item(i);
    std::unordered_map<std::string, std::string> tags =
        utils::getWayTags(way_node);

    // Skip ways that are nothighway. (e.g. waterway, park ... etc)
    if (tags.find("highway") == tags.end()) {
      continue;
    }
    highway = tags["highway"];
    bool is_one_way = false;
    if (tags.find("oneway") != tags.end() && tags["oneway"] == "yes") {
      is_one_way = true;
    }
    std::vector<int64_t> node_ids = utils::getWayNodes(way_node);
    int64_t src_id, dest_id;
    for(int i = 0; i < node_ids.size() - 1; i++) {
      src_id = node_ids[i];
      dest_id = node_ids[i+1];
      nodes.find(src_id)->second.AddArc(dest_id, utils::toEnum(highway));
    }

    // TODO(hcchao@): calculates arc cost
  }


  return true;
}


} // namespace hcchao


