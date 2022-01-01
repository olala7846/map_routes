#include <iostream>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/PlatformUtils.hpp>

using namespace xercesc;

int main(int argc, char* arvg[]) {
  std::cout << "Hello World from main!" << std::endl;

  try {
    XMLPlatformUtils::Initialize();
    std::cout << "initialize successfully!" << std::endl;
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    std::cout << "Error during initialization! :\n"
              << message << "\n";
    XMLString::release(&message);
    return 1;
  }

  // Do your actual work with Xerces-C++ here.
  XercesDOMParser* parser = new XercesDOMParser();
  parser->setValidationScheme(XercesDOMParser::Val_Always);
  parser->setDoNamespaces(true);  // optional

  ErrorHandler* errHandler = (ErrorHandler*) new HandlerBase();
  parser->setErrorHandler(errHandler);

  char const * xmlFile = "taipei_daan_map.osm";
  try {
    parser->parse(xmlFile);
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    std::cout << "Exception message is: \n" << message << "\n";
    XMLString::release(&message);
    return -1;
  }
  catch (const DOMException& toCatch) {
    char* message = XMLString::transcode(toCatch.msg);
    std::cout << "Exception message is: \n" << message << "\n";
    XMLString::release(&message);
    return -1;
  }
  catch (...) {
    std::cout << "Unexpected Exception \n";
    return -1;
  }

  DOMDocument* document = parser->getDocument();
  XMLCh osmNodeTagName[10];
  XMLString::transcode("node", osmNodeTagName, 9);
  DOMNodeList* node_list = document->getElementsByTagName(osmNodeTagName);
  for (XMLSize_t i = 0; i < node_list->getLength(); i++) {
    DOMNode* osmNode = node_list->item(i);
    std::cout << "found node";
  }


  // TODO(hcchao@): RAII pattern for XML parser
  delete parser;
  delete errHandler;

  XMLPlatformUtils::Terminate();
  // Other terminations and cleanup.
  return 0;
}
