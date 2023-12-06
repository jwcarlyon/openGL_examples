#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/OpenGL_config_CLASS.hpp"
#include "../src/Model_CLASS.hpp"
//g++ GTestMain.cpp Camera_GTEST_TEST.cpp Mesh_GTEST_TEST.cpp Model_GTEST_TEST.cpp OpenGL_config_GTEST_TEST.cpp Shader_GTEST_TEST.cpp ../src/Mesh_CLASS.cpp ../src/Model_CLASS.cpp ../src/Shader_CLASS.cpp ../src/OpenGL_config_CLASS.cpp glad.c -lgtest -lgmock -pthread -ldl -lglfw3 -lX11 -o test_case
using namespace testing;

class MeshTEST :public::Test {
public:
    const int MAX_BONE_WEIGHTS = 4;
protected:
    MeshTEST() {}

    Mesh* mesh;
    vector<Texture> textures_loaded;	// stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.

    virtual ~MeshTEST() {}
    virtual void SetUp()
    {
        openGL_config = new OpenGL_config();
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(
            "bulb.obj",
            aiProcess_CalcTangentSpace | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes
              | aiProcess_Triangulate
        );
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "MeshTEST - ERROR::ASSIMP::ReadFile failed {"
              << importer.GetErrorString() << "}\nScene textures: " << scene->mNumTextures
              << "\nScene meshes: " << scene->mNumMeshes << endl;
            return;
        }
        aiNode* node = scene->mRootNode;
        if(0 < node->mNumMeshes)
        {
            aiMesh* assimpMesh = scene->mMeshes[ node->mMeshes[0] ];
            Mesh processedMesh = processMesh(assimpMesh, scene);
            mesh = &processedMesh;
        }
    }
    virtual void TearDown() { openGL_config->destroy(); }
private:
    string directory;
    int m_BoneCount = 0;
    map<string, BoneInfo> m_OffsetMatMap;
    OpenGL_config* openGL_config;

    virtual void extractBoneWeightForVertices(std::vector<Vertex>& vertices, aiMesh* mesh, const aiScene* scene)
    {
        std::cout << "MeshTEST - extractBoneWeightForVertices" << std::endl;
        auto& boneInfoMap = m_OffsetMatMap;
        int& boneCount = m_BoneCount;

        for (int boneIndex = 0; boneIndex < mesh->mNumBones; ++boneIndex)
        {
            int boneID = -1;
            std::string boneName = mesh->mBones[boneIndex]->mName.C_Str();
            if (boneInfoMap.find(boneName) == boneInfoMap.end()){
                BoneInfo newBoneInfo;
                newBoneInfo.id = boneCount;
                newBoneInfo.offset = aiMatrix4x4ToGlmMat4(mesh->mBones[boneIndex]->mOffsetMatrix);
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
                setVertexBoneData(vertices[vertexId], boneID, weight);
            }
        }
    }
    static inline glm::vec3 getGLMVec(const aiVector3D& given_aiVector3D)
    {
        return glm::vec3(given_aiVector3D.x, given_aiVector3D.y, given_aiVector3D.z);
    }
    virtual vector<Texture> loadMaterialTextures(aiMaterial* given_aiMaterial_ptr, aiTextureType given_aiTextureType, string typeName)
    {
        std::cout << "MeshTEST - loadMaterialTextures for typeName: " << typeName << std::endl;
        vector<Texture> textures;
        for(unsigned int textureTypeCount = 0; textureTypeCount < given_aiMaterial_ptr->GetTextureCount(given_aiTextureType); textureTypeCount++)
        {
            aiString texturePath_aiString;
            given_aiMaterial_ptr->GetTexture(given_aiTextureType, textureTypeCount, &texturePath_aiString);
            const char* texturePath_C_Str_ptr = texturePath_aiString.C_Str();
            bool skip = false;
            for(unsigned int textureCount = 0; textureCount < textures_loaded.size(); textureCount++)
            {
                if(std::strcmp(textures_loaded[textureCount].path.data(), texturePath_C_Str_ptr) == 0)
                {
                    textures.push_back(textures_loaded[textureCount]);
                    skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                    break;
                }
            }
            if(!skip)
            {   // if texture hasn't been loaded already, load it
                Texture texture;
                texture.id = textureFromFile(texturePath_C_Str_ptr, this->directory, false);
                texture.type = typeName;
                texture.path = texturePath_C_Str_ptr;
                textures.push_back(texture);
                textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
            }
        }
        return textures;
    }
    virtual Mesh processMesh(aiMesh* mesh, const aiScene* scene)
    {
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        for(unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            for(int i = 0; i < MAX_BONE_WEIGHTS; i++)
            {
                vertex.m_BoneIDs[i] = -1;
                vertex.m_Weights[i] = 0.0f;
            }
            glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
            vertex.Position = getGLMVec(mesh->mVertices[i]);
            vertex.Normal = getGLMVec(mesh->mNormals[i]);
            if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
            {
                glm::vec2 vec;
                vec.x = mesh->mTextureCoords[0][i].x;
                vec.y = mesh->mTextureCoords[0][i].y;
                vertex.TexCoords = vec;
            } else {
                vertex.TexCoords = glm::vec2(0.0f, 0.0f);
                vector.x = mesh->mTangents[i].x;
                vector.y = mesh->mTangents[i].y;
                vector.z = mesh->mTangents[i].z;
                vertex.Tangent = vector;
                vector.x = mesh->mBitangents[i].x;
                vector.y = mesh->mBitangents[i].y;
                vector.z = mesh->mBitangents[i].z;
                vertex.Bitangent = vector;//Should this be just vector.xy?
            }
            vertices.push_back(vertex);
        }
        for(unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            for(unsigned int j = 0; j < face.mNumIndices; j++){
                indices.push_back(face.mIndices[j]);
            }
        }
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
        vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
        std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
        textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
        std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
        textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
        extractBoneWeightForVertices(vertices, mesh, scene);
        return Mesh(vertices, indices, textures);
    }
    virtual void setVertexBoneData(Vertex& vertex, int boneID, float weight)
    {
        // std::cout << "Model_CLASS - setVertexBoneData for boneID: " << boneID << std::endl;
      	for(int i = 0; i < MAX_BONE_WEIGHTS; ++i)
        {
        		if(vertex.m_BoneIDs[i] < 0)
            {
          			vertex.m_Weights[i] = weight;
          			vertex.m_BoneIDs[i] = boneID;
          			break;
        		}
      	}
    }
};

TEST_F(MeshTEST, WhenCreatedThenHasCorrectData)
{
    EXPECT_TRUE(mesh->vertices.size() > 0);
    EXPECT_TRUE(mesh->indices.size() > 0);
}
