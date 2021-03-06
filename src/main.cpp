#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "glm/gtx/string_cast.hpp"

#include "shaders/shader.hpp"
#include "shaders/texture.hpp"
#include "camera/camera.hpp"
#include "scene/scene.hpp"
#include "geom/geom.hpp"
#include "viewer/viewer.hpp"
#include "grid/Grid.h"
#include "solvers/FluidSolver.hpp"
#include "solvers/FlipSolver.h"
#include "solvers/Particle.hpp"

GLFWwindow* window;

//set json data as variables for rendering
float boxScaleX;// = jsonFloats.at(0);
float boxScaleY;// = jsonFloats.at(1);
float boxScaleZ;// = jsonFloats.at(2);
glm::mat4 boxScale;

Camera* camera;
FluidSolver* fluidsolver = new FluidSolver();
Geom* containerGeom;
FlipSolver* flipsolver;


void setSceneParameters() {
    std::vector<float> jsonFloats = loadJSON("../src/scene/scene.json");

    int boxSize = 4;
   
    boxScaleX = boxSize; //jsonFloats.at(0);
    boxScaleY = boxSize; //jsonFloats.at(1);
    boxScaleZ = boxSize; //jsonFloats.at(2);
        

    
    std::cout << "box scale " << glm::to_string(glm::vec3(boxScaleX, boxScaleY , boxScaleZ)) << std::endl;
    boxScale = glm::scale(glm::mat4(1.0f), glm::vec3(boxScaleX, boxScaleY , boxScaleZ));
    containerGeom = new Geom(boxScaleX, boxScaleY, boxScaleZ);
    flipsolver = new FlipSolver(boxScaleX, boxScaleY , boxScaleZ);
    
    float pboundX = boxScaleX -1; //jsonFloats.at(3);
    float pboundY = boxScaleY -1; //jsonFloats.at(4);
    float pboundZ = boxScaleZ -1; //jsonFloats.at(5)
    
    //set  bounds of particles
    //flipsolver->setParticleBounds(pboundX, pboundY, pboundZ);
    
}

void checkKeysPressed() {
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
       // flipsolver->enableGravity();
    }
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS &&
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        //flipsolver->disableGravity();
    }
    if (glfwGetKey(window, GLFW_KEY_E ) == GLFW_PRESS &&
        glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        exportToHoudini();
    }
}



int main( void )
{
    
    //set up FLIPsolver from JSON file
    setSceneParameters();
    
    //set up viewer
    window = setUpWindow();
    if (!window) { return -1; }

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    
    ///////stuff for the particle shader
    // Create and compile our GLSL program from the shaders for the particles
    GLuint programID = LoadShaders( "../src/shaders/Particle.vertexshader", "../src/shaders/Particle.fragmentshader" );
    // Vertex shader
    GLuint CameraRight_worldspace_ID  = glGetUniformLocation(programID, "CameraRight_worldspace");
    GLuint CameraUp_worldspace_ID  = glGetUniformLocation(programID, "CameraUp_worldspace");
    GLuint ViewProjMatrixID = glGetUniformLocation(programID, "VP");
    
    // fragment shader
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");
    
    ///////stuff for the box shader
    GLuint boxprogramID = LoadShaders( "../src/shaders/TransformVertexShader.vertexshader", "../src/shaders/ColorFragmentShader.fragmentshader");
    
    GLuint gridprogramID = LoadShaders( "../src/shaders/TransformVertexShader.vertexshader", "../src/shaders/ColorFragmentShader.fragmentshader");
    
    // box vertex data
    static const GLfloat g_box_vertex_buffer_data[] = {
        -1.0f,-1.0f,-1.0f, //bottom face
        -1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        1.0f, -1.0f, 1.0f,
        1.0f, -1.0f, 1.0f,
        1.0f, -1.0f,-1.0f,
        1.0f, -1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f,1.0f,-1.0f, //top face
        -1.0f,1.0f, 1.0f,
        -1.0f,1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        -1.0f,1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,//corner edges
        -1.0f, 1.0f,-1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f,1.0f, 1.0f,
        1.0f, -1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, -1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        
    };
    
    int size = sizeof(g_box_vertex_buffer_data);
    
    GLfloat g_box_color_buffer_data[size];
    for (int i = 0; i < size; i++) {
        g_box_color_buffer_data[i] = 1.0f;
    }
    
    
    GLuint boxvertexbuffer;
    glGenBuffers(1, &boxvertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, boxvertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_box_vertex_buffer_data),
                    g_box_vertex_buffer_data, GL_STATIC_DRAW);
    
    GLuint boxcolorbuffer;
    glGenBuffers(1, &boxcolorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, boxcolorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_box_color_buffer_data),
                    g_box_color_buffer_data, GL_STATIC_DRAW);
    
    ///////back to particles
    GLuint Texture = loadDDS("../src/shaders/particle.DDS");
    
    ///////put all particles on the viewer
    flipsolver->setUpParticleBuffers();
    
    //////put all particles in data
    flipsolver->Init();
    
    double lastTime = glfwGetTime();
    do
    {
        
        flipsolver->FlipUpdate();
        
        
        // Clear the screen
        glClearColor(0.9, 0.5, 0.8, 0.2);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
       
        
        double currentTime = glfwGetTime();
        //double delta = 1; //currentTime - lastTime;

        camera->computeMatricesFromInputs();
        glm::mat4 ProjectionMatrix = camera->getProjectionMatrix();
        glm::mat4 ViewMatrix = camera->getViewMatrix();
        glm::vec3 CameraPosition(glm::inverse(ViewMatrix)[3]);
        glm::mat4 ViewProjectionMatrix = ProjectionMatrix * ViewMatrix;
    
        //////update grid from particles, then particles from grid
       
        flipsolver->update(DELTA_TIME, boxScaleX, boxScaleY, boxScaleZ, CameraPosition);
        
        //////update where the particles should draw on the page
        flipsolver->updateParticleBuffers();

        // Use our shader
        glUseProgram(programID);
        
        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        glUniform1i(TextureID, 0);
        
        ////// draw the particles
        glUniform3f(CameraRight_worldspace_ID, ViewMatrix[0][0], ViewMatrix[1][0], ViewMatrix[2][0]);
        glUniform3f(CameraUp_worldspace_ID   , ViewMatrix[0][1], ViewMatrix[1][1], ViewMatrix[2][1]);
        glUniformMatrix4fv(ViewProjMatrixID, 1, GL_FALSE, &ViewProjectionMatrix[0][0]);
        
        flipsolver->drawParticles();
        
        ////// draw the box //////
        glUseProgram(boxprogramID);
        
        // Compute the MVP matrix from keyboard and mouse input
        glm::mat4 MVP = camera->getMVPFromInputs(boxScale);
        containerGeom->drawBox(MVP, boxprogramID, boxvertexbuffer, boxcolorbuffer);
        
        checkKeysPressed();
        
        
        ////// render the MACGrid faces for testing //////
        glUseProgram(gridprogramID);
        glm::mat4 MVPGrid = camera->getMVPFromInputs(boxScale);
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    
    while(glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS
          && glfwWindowShouldClose(window) == 0 );

    flipsolver->deleteVBOS();
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);
    glfwTerminate();
    
    return 0;
}



