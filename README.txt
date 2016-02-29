Fluid Solver
Nancy Huang
CIS 563


Controls
The viewport follows the mouse position.
UP/DOWN - Zoom
LEFT/RIGHT - Pan

Main - Calls all the classes in order to set up the scene.

Viewer - Sets up the GLFW window. Adapted from OpenGL’s First Window tutorial.

Shader - LoadShaders() function from OpenGL-Tutorial’s demo files. LoadShaders() Supports reading in files to use as shader programs.

Texture - Reads in image files for use as mesh textures. Copied from OpenGL-Tutorial.

Geom - drawing and VBO setup of the geometry, VBO setup copied from OpenGL-Tutorial.

Scene - Reads in a json file containing scene parameters using jsoncpp. Followed a jsoncpp tutorial to model my code after.

Camera - Controls camera movements and recaculates view matrices while taking in user input. Adapted from OpenGL-Tutorial’s control.cpp.

FluidSolver - Takes care of collision detection (with color change), particle generation, and updating particles based on physics (just gravity for now). Adapted from OpenGL-Tutorial’s Particles tutorial demo files.

Particle - Creates a base mesh and draws the base mesh as instances in order to efficiently render many particle images. Adapted from OpenGL-Tutorial's Particle tutorial demo files and refactored into its own class.




