#ifndef MODEL_CLASS_HPP_INCLUDED
#define MODEL_CLASS_HPP_INCLUDED

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem; //linux install ghc in bash
#endif
#include <assimp/config.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
// #pragma once
// #include<assimp/quaternion.h>
// #include<assimp/vector3.h>
// #include<assimp/matrix4x4.h>
#include <fstream>
#include <ghc/filesystem.hpp>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <mutex>

#include "Mesh_CLASS.hpp"

using namespace std;

class Model
{
public:
    Model(string const &path, bool gamma = false);

    const int MAX_BONE_WEIGHTS = 4;
    bool gammaCorrection;
    vector<Mesh> meshes;
    vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.

    void draw(Shader shader);
    int& getBoneCount();
    string getDirectory();
    auto& getOffsetMatMap();
    Mesh& getMeshGeometry_synchronized();
private:
    string directory;
    int m_BoneCount = 0;
    map<string, BoneInfo> m_OffsetMatMap;
    std::mutex meshMutex;

    void extractBoneWeightForVertices(vector<Vertex>& vertices, aiMesh* mesh, const aiScene* scene);
    vector<Texture> loadMaterialTextures(aiMaterial* given_aiMaterial_ptr, aiTextureType given_aiTextureType, string typeName);
    void loadModel(string const& path);//shouldnt this be const string& path?
    Mesh processMesh(aiMesh* mesh, const aiScene* scene);
    void processNode(aiNode* node, const aiScene* scene);
    void setDirectory(string const& updatedPath);
    void setVertexBoneData(Vertex& vertex, int boneID, float weight);
    void setVertexBoneDataToDefault(Vertex& vertex);
};

glm::mat4 aiMatrix4x4ToGlmMat4(const aiMatrix4x4& given_aiMatrix4x4);
string getExePathWstring();
static inline glm::vec3 getGLMVec(const aiVector3D& given_aiVector3D);
ostream& operator<<(ostream& os, const Mesh& mesh);
void setOsCorrectPath(string& path);
unsigned int textureFromFile(const char* path, const string&  directory, bool gamma);

#endif // MODEL_CLASS_HPP_INCLUDED
