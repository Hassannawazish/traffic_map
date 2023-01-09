# Installation Guide
Downloading GLFW and GLEW

git submodule add https://github.com/glfw/glfw
git submodule add https://github.com/Perlmint/glew-cmake glew

or try this.
git submodule add -f https://github.com/glfw/glfw
git submodule add -f https://github.com/Perlmint/glew-cmake glew

In main code, GLEW need to be added before GLFW. Otherwise it won't work.
