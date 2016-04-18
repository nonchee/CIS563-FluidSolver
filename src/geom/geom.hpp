//geom.hpp
#pragma once
#include <vector>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "glm/gtx/string_cast.hpp"
//using namespace glm;

//std::vector<float> boxBounds;

class Geom {
    float boxBoundX;
    float boxBoundY;
    float boxBoundZ;
public:    
    Geom(float bbX, float bbY, float bbZ); 
    std::vector<float> geomVertices;
    
    
    void drawBox(glm::mat4 MVP, GLuint boxprogramID, GLuint boxvertexbuffer, GLuint boxcolorbuffer);
    bool insideContainer(glm::vec3 pos);
    //bool insidecontainer(Particle p);
};


