#include "scene.hpp"



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
        printf("Impossible to open %s. Are you in the right directory ? Don't forget to read the FAQ !\n", json_file_path);
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
    
    //openvdb::initialize();
    std::cout << " exporting to houdini!" << std::endl;
    
    
    
}