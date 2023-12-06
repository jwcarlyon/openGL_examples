#ifndef MESH_CLASS_HPP_INCLUDED
#define MESH_CLASS_HPP_INCLUDED

#define MAX_BONE_INFLUENCE 4

#include <glad/glad.h> // holds all OpenGL type declarations
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Shader_CLASS.hpp"
using namespace std;

struct BoneInfo
{
    // For uniquely indentifying the bone and for indexing bone transformation in shaders
    int id;
    // map from bone name to offset matrix. offset matrix transforms bone from bone space to local space
    glm::mat4 offset;
};

struct Texture {
    unsigned int id;
    string type;
    string path;
};

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
    glm::vec3 Tangent;
    glm::vec3 Bitangent;
    //bone indexes which will influence this vertex
    int m_BoneIDs[MAX_BONE_INFLUENCE];
    //weights from each bone
    float m_Weights[MAX_BONE_INFLUENCE];
};

class Mesh {
public:
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures);

    vector<unsigned int> indices;
    vector<Texture> textures;
    vector<Vertex> vertices;
    unsigned int VAO;

    void draw(Shader shader);
private:
    unsigned int VBO, EBO;
    // initializes all the buffer objects/arrays
    void setupMesh();
};

#endif // MESH_CLASS_HPP_INCLUDED
