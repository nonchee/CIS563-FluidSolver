//
//  camera.hpp
//  Thanda
//

#ifndef camera_hpp
#define camera_hpp

class Camera {

public:
    
    Camera(GLFWwindow* g);
    void computeMatricesFromInputs();
    glm::mat4 getViewMatrix();
    glm::mat4 getProjectionMatrix();
    glm::mat4 getMVPFromInputs(glm::mat4 boxScale);
    
};


#endif /* camera_hpp */
