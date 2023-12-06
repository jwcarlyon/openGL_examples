#ifndef SHADER_CLASS_HPP_INCLUDED
#define SHADER_CLASS_HPP_INCLUDED

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "WireframeCollector.h"

class Shader
{
public:

    Shader(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr);

    Shader() {};

    unsigned int ID;

    void drawTexturedSkybox();
    void checkCompileErrors(GLuint shader, std::string type);
    std::string getVertexShader();
    std::string getFragmentShader();
    void use();
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec2(const std::string &name, const glm::vec2 &value) const;
    void setVec2(const std::string &name, float x, float y) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    void setVec4(const std::string &name, const glm::vec4 &value) const;
    void setVec4(const std::string &name, float x, float y, float z, float w);
    void setMat2(const std::string &name, const glm::mat2 &mat) const;
    void setMat3(const std::string &name, const glm::mat3 &mat) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;
private:
    std::string fragment_shader_str;
    std::string vertex_shader_str;

    std::string setFragmentShader(const char* fragmentPath);
    std::string setVertexShader(const char* vertexPath);
};

class LineShader : public Shader
{
public:
    LineShader();

    void draw(std::vector<Line>& lineVector);
private:
    unsigned int VAO, VBO;
    // void checkCompileErrors(GLuint shader, std::string type);
};

class SkyboxShader : public Shader
{
public:
    SkyboxShader(const char* vertexPath, const char* fragmentPath);

    void draw(glm::mat4 projection, glm::mat4 view);
    unsigned int loadCubeMap();
private:
    unsigned int cubemapTexture, VAO, VBO;
    std::vector<std::string> faces;

    unsigned int loadCubemap(std::vector<std::string> faces);
};

#endif // SHADER_CLASS_HPP_INCLUDED
