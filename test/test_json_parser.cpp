#include <mrta_utilities/mrta_json_parser.h>

int main(int argc, char const *argv[])
{
  MrtaConfig json_config = MrtaJsonParser::parseJsonFile("experiments/testing_setups/test_json_parser.json");

  return 0;
}


