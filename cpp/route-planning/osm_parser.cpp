#include "osm_parser.h"

#include <iostream>
#include <sstream>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/PlatformUtils.hpp>

namespace hcchao
{

using xercesc::XMLException;
using xercesc::XMLString;

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
}

bool OSMParser::ParseNodes() {
  if (!initialized_)
    return false;

  // Initialize XML parser
  std::unique_ptr<xercesc::XercesDOMParser> parser(new xercesc::XercesDOMParser());

  parser->setValidationScheme(xercesc::XercesDOMParser::Val_Always);
  parser->setDoNamespaces(true);  // optional

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
  catch (const xercesc::DOMException& toCatch) {
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
  XMLCh osmNodeTagName[10];
  XMLString::transcode("node", osmNodeTagName, 9);
  xercesc::DOMNodeList* node_list = document->getElementsByTagName(osmNodeTagName);
  for (XMLSize_t i = 0; i < node_list->getLength(); i++) {
    xercesc::DOMNode* osmNode = node_list->item(i);
    // TODO(hcchao@): parse nodes and store it in an collection (e.g. vector)
    std::cout << "found node";
  }

  return {};
}

// std::vector<std::unique_ptr<MapNode>> OSMParser::GetParsedNodes() {
//   return std::move(parsed_nodes_);
// }


} // namespace hcchao


