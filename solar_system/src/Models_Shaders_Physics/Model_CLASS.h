#ifndef MODEL_H
#define MODEL_H
#include <assimp/config.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem; //linux install ghc in bash
#endif
#include <fstream>
#include <ghc/filesystem.hpp>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <map>
#include <Models_Shaders_Physics/Assimp_GLM_Helpers_CLASS.h>
#include <Models_Shaders_Physics/Bone_Info_CLASS.h>
#include <Models_Shaders_Physics/Mesh_CLASS.h>
#include <Models_Shaders_Physics/Shader_CLASS.h>
#include <sstream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <string>
using namespace std;
//TODO: decide if this function is necessary for the scene engine
string getExePathWstring();
void setOsCorrectPath(string& path);
unsigned int TextureFromFile(const char *path, const string &directory, bool gamma = false);

class Model
{
public:
    vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    vector<Mesh> meshes;
    string directory;
    bool gammaCorrection;
    const int MAX_BONE_WEIGHTS = 4;

    // constructor, expects a filepath to a 3D model.
    Model(string const &path, bool gamma = false) : gammaCorrection(gamma)
    {
        loadModel(path);
    }

    // draws the model, and thus all its meshes
    void Draw(Shader shader)
    {
        for(unsigned int i = 0; i < meshes.size(); i++){
            meshes[i].Draw(shader);
        }
    }

    auto& GetOffsetMatMap()
    {
      return m_OffsetMatMap;
    }

    int& GetBoneCount()
    {
      return m_BoneCount;
    }

private:
    std::map<string, BoneInfo> m_OffsetMatMap;
    int m_BoneCount = 0;

    void ExtractBoneWeightForVertices(std::vector<Vertex>& vertices, aiMesh* mesh, const aiScene* scene)
    {
      // std::cout << "Model_CLASS - ExtractBoneWeightForVertices" << std::endl;
      auto& boneInfoMap = m_OffsetMatMap;
      int& boneCount = m_BoneCount;

      for (int boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
      {
        int boneID = -1;
        std::string boneName = mesh->mBones[boneIndex]->mName.C_Str();
        if (boneInfoMap.find(boneName) == boneInfoMap.end()){
          BoneInfo newBoneInfo;
          newBoneInfo.id = boneCount;
          newBoneInfo.offset = AssimpGLMHelpers::ConvertMatrixToGLMFormat(mesh->mBones[boneIndex]->mOffsetMatrix);
          boneInfoMap[boneName] = newBoneInfo;
          boneID = boneCount;
          boneCount++;
        } else {
          boneID = boneInfoMap[boneName].id;
        }
        assert(boneID != -1);
        auto weights = mesh->mBones[boneIndex]->mWeights;
        int numWeights = mesh->mBones[boneIndex]->mNumWeights;

        for (int weightIndex = 0; weightIndex < numWeights; ++weightIndex)
        {
          int vertexId = weights[weightIndex].mVertexId;
          float weight = weights[weightIndex].mWeight;
          assert(vertexId <= vertices.size());
          SetVertexBoneData(vertices[vertexId], boneID, weight);
        }
      }
    }
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    // checks all material textures of a given type and loads the textures if they're not loaded yet.
    // the required info is returned as a Texture struct.
    vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName)
    {
        // std::cout << "Model_CLASS - loadMaterialTextures for typeName: " << typeName << std::endl;
        vector<Texture> textures;
        for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
        {
            aiString str;
            mat->GetTexture(type, i, &str);
            // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
            bool skip = false;
            for(unsigned int j = 0; j < textures_loaded.size(); j++)
            {
                if(std::strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0)
                {
                    textures.push_back(textures_loaded[j]);
                    skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                    break;
                }
            }
            if(!skip){   // if texture hasn't been loaded already, load it
                Texture texture;
                texture.id = TextureFromFile(str.C_Str(), this->directory);
                texture.type = typeName;
                texture.path = str.C_Str();
                textures.push_back(texture);
                textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
            }
        }
        return textures;
    }

    void loadModel(string const &path)
    {
        // read file via ASSIMP
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(
          path,
          aiProcess_CalcTangentSpace |
          aiProcess_OptimizeGraph |
          aiProcess_OptimizeMeshes |
          aiProcess_Triangulate
        );
        // check for errors
        // std::cout << "Model_CLASS - loadModel @path: " << path << std::endl
        //   << "Checking for problems: " << importer.GetErrorString() << std::endl;
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "Model_CLASS - ERROR::ASSIMP:: " << importer.GetErrorString()
            << "\nScene textures: " << scene->mNumTextures
            << "\nScene animations: " << scene->mNumMeshes << endl;
            return;
        }
        // retrieve the directory path of the filepath
        directory = path.substr(0, path.find_last_of('/'));

        // process ASSIMP's root node recursively
        processNode(scene->mRootNode, scene);
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene)
    {
        // std::cout << "Model_CLASS - Initiate processMesh # of vertecies: " << mesh->mNumVertices << std::endl;
        // data to fill
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        // Walk through each of the mesh's vertices
        for(unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            SetVertexBoneDataToDefault(vertex);
            glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
            vertex.Position = AssimpGLMHelpers::GetGLMVec(mesh->mVertices[i]);
            vertex.Normal = AssimpGLMHelpers::GetGLMVec(mesh->mNormals[i]);
            // // positions
            // vector.x = mesh->mVertices[i].x;
            // vector.y = mesh->mVertices[i].y;
            // vector.z = mesh->mVertices[i].z;
            // vertex.Position = vector;
            // // normals
            // vector.x = mesh->mNormals[i].x;
            // vector.y = mesh->mNormals[i].y;
            // vector.z = mesh->mNormals[i].z;
            // vertex.Normal = vector;
            // texture coordinates
            if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
            {
              glm::vec2 vec;
              // a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't
              // use models where a vertex can have multiple texture coordinates so we always take the first set (0).
              vec.x = mesh->mTextureCoords[0][i].x;
              vec.y = mesh->mTextureCoords[0][i].y;
              vertex.TexCoords = vec;
            } else {
              // std::cout << "Model_CLASS - mesh snapshot: " << mesh << std::endl;
              // std::cout << "Model_CLASS - texture coordinates not found.\n-Mesh must have defined mTangents and mBitangents or segmentation fault will occur." << std::endl;
              vertex.TexCoords = glm::vec2(0.0f, 0.0f);
              // tangent
              vector.x = mesh->mTangents[i].x;
              vector.y = mesh->mTangents[i].y;
              vector.z = mesh->mTangents[i].z;
              vertex.Tangent = vector;
              // bitangent
              vector.x = mesh->mBitangents[i].x;
              vector.y = mesh->mBitangents[i].y;
              vector.z = mesh->mBitangents[i].z;
              vertex.Bitangent = vector;//Should this be just vector.xy?
            }
            vertices.push_back(vertex);
        }
        // std::cout << "Model_CLASS - processMesh vertecies processed" << std::endl;
        // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
        for(unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            // retrieve all indices of the face and store them in the indices vector
            for(unsigned int j = 0; j < face.mNumIndices; j++){
                indices.push_back(face.mIndices[j]);
            }
        }
        // std::cout << "Model_CLASS - processMesh faces processed" << std::endl;
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        // we assume a convention for sampler names in the shaders. Each diffuse texture should be named
        // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER.
        // Same applies to other texture as the following list summarizes:
        // diffuse: texture_diffuseN
        // specular: texture_specularN
        // normal: texture_normalN
        // 1. diffuse maps
        vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
        // 2. specular maps
        vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
        // 3. normal maps
        std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
        textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
        // 4. height maps
        std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
        textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
        ExtractBoneWeightForVertices(vertices,mesh,scene);
        // return a mesh object created from the extracted mesh data
        return Mesh(vertices, indices, textures);
    }

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene)
    {
        // std::cout << "Model_CLASS - processNode" << std::endl;
        // process each mesh located at the current node
        for(unsigned int i = 0; i < node->mNumMeshes; i++)
        {
            // the node object only contains indices to index the actual objects in the scene.
            // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(mesh, scene));
        }
        // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
        for(unsigned int i = 0; i < node->mNumChildren; i++)
        {
            processNode(node->mChildren[i], scene);
        }
    }

    void SetVertexBoneData(Vertex& vertex, int boneID, float weight)
  	{
      // std::cout << "Model_CLASS - SetVertexBoneData for boneID: " << boneID << std::endl;
  		for (int i = 0; i < MAX_BONE_WEIGHTS; ++i){
  			if (vertex.m_BoneIDs[i] < 0){
  				vertex.m_Weights[i] = weight;
  				vertex.m_BoneIDs[i] = boneID;
  				break;
  			}
  		}
  	}

    void SetVertexBoneDataToDefault(Vertex& vertex)
    {
      for (int i = 0; i < MAX_BONE_WEIGHTS; i++){
        vertex.m_BoneIDs[i] = -1;
        vertex.m_Weights[i] = 0.0f;
      }
    }
};

unsigned int TextureFromFile(const char *path, const string &directory, bool gamma)
{
    string filename = string(path);
    filename = directory + '/' + filename;
    unsigned int textureID;
    glGenTextures(1, &textureID);
    setOsCorrectPath(filename);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    // std::cout << "Model_CLASS - TextureFromFile path: " << filename.c_str() << "\n@directory: " << directory << std::endl;
    if (data)
    {
        GLenum format;
        if (nrComponents == 1){
            format = GL_RED;
        } else if (nrComponents == 3){
            format = GL_RGB;
        } else if (nrComponents == 4){
            format = GL_RGBA;
        }
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        // std::cout << "Model_CLASS - Texture loaded" << std::endl;
    } else {
        // std::cout << "Model_CLASS - Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }
    return textureID;
}

string getExePathWstring()
{
  return fs::current_path().string();
}

ostream& operator<<(ostream& os, const Mesh& mesh) {
  return os << "VAO: " << mesh.VAO << endl;
}

void setOsCorrectPath(string& path)
{
  string conventional_path;
  #ifdef _WIN32
  // std::cout << "Model_CLASS WINDOWS - setOsCorrectPath filename before: " << path << std::endl;
  replace(path.begin(), path.end(), '/', '\\');
  if(path.find('.'))
  {//removing any filetypes in a directory... this is dumb, but usable for debugging
    path.erase(path.find('.'));
    path.append("\\");
  }
  #endif
  #ifdef linux
  // std::cout << "Model_CLASS LINUX - setOsCorrectPath filename before: " << path << std::endl;
  if(path.find_first_of("/"))
  {
    conventional_path = path.substr(0, path.find_first_of("/"));
    conventional_path = conventional_path + "/texture/texture.png";
    path = conventional_path;
  }
  #endif
  // std::cout << "Model_CLASS - setOsCorrectPath filename after: " << path << std::endl;
}
#endif
