#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <iostream>

#include <json/json.h>
#include <openexr/OpenEXRConfig.h>
#include <boost/config.hpp>
#include <openvdb/openvdb.h>

std::vector<float> loadJSON(const char * vertex_file_path);
void exportToHoudini();