#include <iostream>

#include "GL/glew.h"
#include "GLFW/glfw3.h"

void setWindowHints(){
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
}

int main()
{
    if(!glfwInit()){
        std::cerr<<" GLFW initialization Failed........!"<<std::endl;
        return -1;  
    }

    setWindowHints();
    GLFWwindow* window = glfwCreateWindow(1280, 720, "FBX Visualizer", nullptr, nullptr);
    if(window == nullptr){
        std::cerr<<" GLFW Window Creation Failed........!"<<std::endl;
        glfwTerminate();
        return -2;  
    }

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK){
        std::cout<<" GLEW Initialization Failed..........!"<<std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -3;
    }

    glClearColor(0.3f,0.3f,0.4f,1.0f);
    while(!glfwWindowShouldClose(window)){
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "OPEN GL "<<std::endl;
    return 0;
}