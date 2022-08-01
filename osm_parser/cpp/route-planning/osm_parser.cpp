#include "osm_parser.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <utility>
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#include "osm_xml_utils.h"

namespace hcchao
{

OSMParser::OSMParser(std::string filename) : filename_(filename) {
  // no longer use Xerces XML parser, use rapidxml instead
  // TODO: we don't really need to init parser with filename
}

OSMParser::~OSMParser() {
}

std::pair<int64_t, MapNode> MakeOsmNodeKeyValuePair(const ::rapidxml::xml_node<> *xml_node) {
  // TODO parse node attributes
  // TODO parse attrobite lat, lon, id
  int64_t nid = 100;

  MapNode new_node(nid, 1.0, 1.0);
  return std::make_pair(nid, new_node);
}

bool OSMParser::ParseXml(std::unordered_map<int64_t, MapNode>& nodes) {

  /*
  std::ifstream xml_file(filename_, std::ios::binary);
  if (!xml_file.is_open()) {
    error_msg_ = "Can't open file.";
    return false;
  } else {
    std::cout << "File " << filename_ << " opened for pasing.";
  }


  // If you want to parse contents of a file, you must first load
  // the file into the memory, and pass pointer to its beginning.
  // Make sure that data is zero-terminated.
  // https://stackoverflow.com/questions/18816126/c-read-the-whole-file-in-buffer
  std::streamsize file_size = xml_file.tellg();
  std::vector<char> buffer(file_size);
  if (!xml_file.read(buffer.data(), file_size)) {
    error_msg_ = "Failed to read file to buffer.";
    return false;
  }
  */

  std::cout << "Will parse document " << filename_ << std::endl;

  // https://stackoverflow.com/questions/29474690/how-do-i-handle-rapidxml-errors
  rapidxml::file<> xml_file(filename_.c_str());
  rapidxml::xml_document<> doc;
  try {
    doc.parse<0>(xml_file.data());
  }
  catch(...) {
    std::cout << "An error occured while parsing XML." << std::endl;
    return false;
  }

  rapidxml::xml_node<> *osm = doc.first_node("osm");
  std::cout<< "osm_node node is: " << osm->name() << std::endl;
  rapidxml::xml_node<> *node = osm->first_node("node");
  // Parse Node
  int num_nodes = 0;
  while(node != 0) { // next_sibling returns 0 if not found
    num_nodes++;
    nodes.emplace(MakeOsmNodeKeyValuePair(node));
    node = node->next_sibling("node");
  }
  std::cout << "Total number of nodes: " << num_nodes;

  // TODO parse way
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

  return true;
}


} // namespace hcchao


