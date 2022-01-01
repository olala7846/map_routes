#ifndef HCCHAO_OSMPARSER_H_
#define HCCHAO_OSMPARSER_H_

#include <string>

namespace hcchao
{

class MapNode {
  public:
    double lat;
    double lng;
};


class OSMParser {
public:
  OSMParser(std::string filename);  // constructor
  ~OSMParser();  // destructor

  // Prohibit copy for RAII object
  OSMParser(const OSMParser&) = delete;  // no copy constructor
  void operator=(const OSMParser&) = delete;  // no copy assignment

  // Parse the OSM file and returns whether the parsing was successful
  // If failed, calls GetErrorMessage function for detailed error message.
  bool ParseNodes();

  // Detailed error message on why an action failed.
  std::string GetErrorMessage() { return error_msg_; }

  // std::vector<std::unique_ptr<MapNode>> GetParsedNodes();

private:
  std::string filename_;  // OSM XML file name
  std::string error_msg_;  // detailed error message if error exists
  // std::vector<std::unique_ptr<MapNode>> parsed_nodes_;
  bool initialized_;

};

} // namespace hcchao



#endif  // HCCHAO_OSMPARSER_H_
