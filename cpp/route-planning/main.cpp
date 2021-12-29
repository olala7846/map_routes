#include <iostream>
#include <xercesc/util/PlatformUtils.hpp>

using namespace xercesc;

int main(int argc, char* arvg[]) {
  std::cout << "Hello World from main!" << std::endl;

  try {
    XMLPlatformUtils::Initialize();
    std::cout << "initialize successfully!" << std::endl;
  }
  catch (const XMLException& toCatch) {
    // Do your failure processing here
    std::cout << "failed to initialize with error" << std::endl;
    return 1;
  }

  // Do your actual work with Xerces-C++ here.

  XMLPlatformUtils::Terminate();

  // Other terminations and cleanup.
  return 0;

}
