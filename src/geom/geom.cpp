// geom.cpp

#include <GL/glew.h>
#include "geom.hpp"

Geom::Geom(float bbX, float bbY, float bbZ) {
    boxBoundX = bbX;
    boxBoundY = bbY;
    boxBoundZ = bbZ;
}

void Geom::drawBox(glm::mat4 MVP, GLuint boxprogramID, GLuint boxvertexbuffer, GLuint boxcolorbuffer) {
        
        
        // Get a handle for our "MVP" uniform
        GLuint boxMatrixID = glGetUniformLocation(boxprogramID, "MVP");
        
        
        // Send our transformation to the currently bound shader, in the "MVP" uniform
        glUniformMatrix4fv(boxMatrixID, 1, GL_FALSE, &MVP[0][0]);
        
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, boxvertexbuffer);
        glVertexAttribPointer(
                              0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                              3,                  // size
                              GL_FLOAT,           // type
                              GL_FALSE,           // normalized?
                              0,                  // stride
                              (void*)0            // array buffer offset
                              );
        
        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, boxcolorbuffer);
        glVertexAttribPointer(
                              1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                              3,                                // size
                              GL_FLOAT,                         // type
                              GL_FALSE,                         // normalized?
                              0,                                // stride
                              (void*)0                          // array buffer offset
                              );
        
        // Draw the cube !
        glLineWidth(3);
        glDrawArrays(GL_LINES, 0, 72); // 8 *3 -> 24 line segments
        
        
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        
        
    }
    
bool Geom::insideContainer(glm::vec3 pos) {
    //if intersect with bottom face
    
    return pos.x > -boxBoundX && pos.x < boxBoundX
        && pos.y > -boxBoundY && pos.y < boxBoundY
           && pos.z > -boxBoundZ && pos.z < boxBoundZ;
}
    
    /*glm::intersectRayPlane	(genType const & 	orig,
                             genType const & 	dir,
                             genType const & 	planeOrig,
                             genType const & 	planeNormal,
                             typename genType::value_type & 	intersectionDistance 
                             )*/
    //if intersect with side faces





