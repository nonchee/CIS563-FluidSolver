#include "scene.hpp"
#include <openvdb/tools/LevelSetSphere.h>


std::vector<float> loadJSON(const char * json_file_path) {
    
    std::vector<float> jsonSceneParameters;
    
    // Read the Vertex Shader code from the file
    std::string JSONData;
    std::ifstream JSONDataStream(json_file_path, std::ios::in);
    if(JSONDataStream.is_open()){
        std::string Line = "";
        while(getline(JSONDataStream, Line))
            JSONData += "\n" + Line;
        JSONDataStream.close();
    } else{
        printf("Impossible to open %s. Are you in the right directory ?\n", json_file_path);
        getchar();
        return jsonSceneParameters;
    }
    
    
    Json::Value root;   // will contains the root value after parsing.
    
    Json::Reader reader;
    bool parsingSuccessful = reader.parse( JSONData, root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration\n" << reader.getFormattedErrorMessages();
        return jsonSceneParameters;
    }
    
    jsonSceneParameters.push_back((root["containerDim"]["scaleX"]).asFloat());
    jsonSceneParameters.push_back((root["containerDim"]["scaleY"]).asFloat());
    jsonSceneParameters.push_back((root["containerDim"]["scaleZ"]).asFloat());
    
    jsonSceneParameters.push_back((root["particleDim"]["boundX"]).asFloat());
    jsonSceneParameters.push_back((root["particleDim"]["boundY"]).asFloat());
    jsonSceneParameters.push_back((root["particleDim"]["boundZ"]).asFloat());
    jsonSceneParameters.push_back((root["particleSeparation"]).asFloat());
    
    return jsonSceneParameters;
    
}

void exportToHoudini() {
    
    openvdb::initialize();
    
    std::cout << " Exporting to VDB! Find it in flippity.vdb" << std::endl;
    
    openvdb::FloatGrid::Ptr grid =
        openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(50.0,
                                                                 openvdb::Vec3f(1.5, 2, 3),
                                                                /*voxel size=*/0.5,
                                                                 /*width=*/4.0);

    // Create a VDB file object.
    openvdb::io::File file("../vdbfiles/flippity.vdb");
    
    // Add the grid pointer to a container.
    openvdb::GridPtrVec grids;
    grids.push_back(grid);
    // Write out the contents of the container.
    file.write(grids);
    file.close();
    
    
}








