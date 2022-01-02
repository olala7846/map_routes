#include "osm_parser.h"

#include <iostream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include <utility>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

namespace hcchao
{

using xercesc::XercesDOMParser;
using xercesc::XMLException;
using xercesc::XMLString;
using xercesc::DOMDocument;
using xercesc::DOMElement;
using xercesc::DOMNode;

OSMParser::OSMParser(std::string filename) : filename_(filename) {
  // TODO(hcchao@): find a better way to initialize XMLPlatformUtils.
  // It is usually not a good idea to run code that can throw
  // exceptions inside constructor, but I haven't find a good way
  // create an RAII that ensures Initialize() and Terminate() being called
  // once properly without doing it in the constructor.
  try {
    xercesc::XMLPlatformUtils::Initialize();
    initialized_ = true;
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    std::stringstream ss;
    ss << "Error during xerces XML initialization! :\n" << message << "\n";
    XMLString::release(&message);
    ss >> error_msg_;
    initialized_ = false;
  }
  catch (...) {
    error_msg_ = "Unknown error during Xercesc initialization.";
    initialized_ = false;
  }
}

OSMParser::~OSMParser() {
  xercesc::XMLPlatformUtils::Terminate();
  // delete parsed_nodes_;
}

// Converts XMLCh* to std::string
std::string toStdString(const XMLCh* raw_xml_str) {
  char* temp = XMLString::transcode(raw_xml_str);
  std::string std_str(temp);
  XMLString::release(&temp);
  return std_str;
}

// Wrapper function for extracting tag name as string, and more
// importantly releasing resource.
// https://stackoverflow.com/questions/9826518/purpose-of-xmlstringtranscode
std::string getElementTagName(DOMElement* elem) {
  return toStdString(elem->getTagName());
}

bool OSMParser::ParseXml(std::unordered_map<int64_t, MapNode>& nodes) {
  if (!initialized_)
    return false;

  // Initialize XML parser
  std::unique_ptr<XercesDOMParser> parser(new XercesDOMParser());

  // Disabling some parser capabilities (e.g. DTD and namespace) to make
  // parsing faster.  See
  // https://wiki.openstreetmap.org/wiki/OSM_XML#Basics
  parser->setValidationScheme(XercesDOMParser::Val_Never);
  parser->setDoNamespaces(false);  // optional

  std::unique_ptr<xercesc::ErrorHandler> errHandler(
      (xercesc::ErrorHandler*) new xercesc::HandlerBase());
  parser->setErrorHandler(errHandler.get());

  // Start parsing
  try {
    parser->parse(filename_.c_str());
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    std::stringstream ss;
    ss << "Exception message is: \n" << message << "\n";
    ss >> error_msg_;
    XMLString::release(&message);
    return false;
  }
  catch (const xercesc::DOMRangeException& toCatch) {
    char* message = XMLString::transcode(toCatch.msg);
    std::stringstream ss;
    ss << "Exception message is: \n" << message << "\n";
    ss >> error_msg_;
    XMLString::release(&message);
    return false;
  }
  catch (...) {
    error_msg_ = "Unexpected Exception \n";
    return false;
  }

  xercesc::DOMDocument* document = parser->getDocument();
  DOMElement* root = document->getDocumentElement();
  std::string root_tag_name = getElementTagName(root);
  if (root_tag_name != "osm") {
    std::stringstream ss;
    ss << "Unexpected root element (" << root_tag_name << ") in XML.";
    ss >> error_msg_;
    return false;
  }

  // First, construct all nodes.
  XMLCh osmNodeTagName[10];
  XMLString::transcode("node", osmNodeTagName, 9);
  xercesc::DOMNodeList* node_list = document->getElementsByTagName(osmNodeTagName);

  XMLCh osmLatAttrName[10];
  XMLString::transcode("lat", osmLatAttrName, 9);
  XMLCh osmLonAttrName[10];
  XMLString::transcode("lon", osmLonAttrName, 9);
  XMLCh osmIdAttrName[10];
  XMLString::transcode("id", osmIdAttrName, 9);

  for (XMLSize_t i = 0; i < node_list->getLength(); i++) {
    DOMElement* osm_node = (DOMElement* )node_list->item(i);
    std::string node_id = toStdString(osm_node->getAttribute(osmIdAttrName));
    std::string lat = toStdString(osm_node->getAttribute(osmLatAttrName));
    std::string lon = toStdString(osm_node->getAttribute(osmLonAttrName));
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


  // XMLCh osmNodeTagName[10];
  // XMLString::transcode("node", osmNodeTagName, 9);
  // xercesc::DOMNodeList* node_list = document->getElementsByTagName(osmNodeTagName);
  // for (XMLSize_t i = 0; i < node_list->getLength(); i++) {
  //   xercesc::DOMNode* osmNode = node_list->item(i);

  //   // std::unique_ptr<MapNode> node_ptr(new MapNode());
  //   parsed_nodes_.emplace_back(new MapNode());
  //   // TODO(hcchao@): parse nodes and store it in an collection (e.g. vector)
  //   std::cout << "found node";
  // }

  return true;
}

// std::vector<std::unique_ptr<MapNode>> OSMParser::GetParsedNodes() {
//   return std::move(parsed_nodes_);
// }


} // namespace hcchao


