#include "MeshLoader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

//#include <iostream>
#include <cassert>

#include <pangolin/image/image_io.h>

namespace MeshLoader {

aiMatrix4x4 getFullT(const std::vector<aiMatrix4x4> &transforms) {
    // identity transform
    aiMatrix4x4 T;
    // concatenate all transformations
    for(auto t : transforms)
        T *= t;
    return T;
}

void getMesh(const aiScene* const scene, const aiNode* const node,
             Mesh &obj_mesh, std::vector<aiMatrix4x4> transforms)
{
    // start to count mesh ID from here
    const unsigned int offset = obj_mesh.vertices.size();

    // we need to skip the transformation from the scene to the root frame
    if(node->mParent != NULL)
        transforms.push_back(node->mTransformation);

    const aiMatrix4x4 T = getFullT(transforms);

    for(uint m=0; m<node->mNumMeshes; m++) {
        const aiMesh* const mesh = scene->mMeshes[node->mMeshes[m]];
        if(mesh->mPrimitiveTypes == aiPrimitiveType_TRIANGLE) {
            if(scene->HasMaterials()) {
                const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

                for(uint iText(0); iText < material->GetTextureCount(aiTextureType_DIFFUSE); iText++) {
                    aiString path;
                    material->GetTexture(aiTextureType_DIFFUSE, iText, &path);
                    //std::cout<<"diffuse texture at: "<<path.C_Str()<<std::endl;
                    obj_mesh.texture = pangolin::LoadImage(obj_mesh.directory+'/'+path.C_Str());
                } // texture
            } // materials

            for(uint v=0; v<mesh->mNumVertices; v++) {
                // transform each vertice
                const aiVector3D vert = T*mesh->mVertices[v];
                obj_mesh.vertices.push_back({vert.x, vert.y, vert.z});

                // colour, r, g, b
                if(mesh->mColors[0]!=NULL) {
                    obj_mesh.colour.push_back({mesh->mColors[0][v].r,
                                               mesh->mColors[0][v].g,
                                               mesh->mColors[0][v].b});
                }

                // texture coordiantes
                // we need to flip the image vertically (y-axis)
                if(mesh->mTextureCoords[0]!=NULL) {
                    obj_mesh.uv.push_back({mesh->mTextureCoords[0][v].x,
                                                1-mesh->mTextureCoords[0][v].y});
                }
            }

            // transform each normal
            if(mesh->HasNormals()) {
                for(uint v=0; v<mesh->mNumVertices; v++) {
                    const aiVector3D norm = T*mesh->mNormals[v];
                    obj_mesh.normals.push_back({norm.x, norm.y, norm.z});
                }
            }

            // get faces and store vertex ID with offsets for each new mesh
            for(unsigned int f=0; f<mesh->mNumFaces; f++) {
                const aiFace face = mesh->mFaces[f];
                assert(face.mNumIndices == 3);
                obj_mesh.faces.push_back({offset + face.mIndices[0],
                                          offset + face.mIndices[1],
                                          offset + face.mIndices[2]});
            }
        } // check aiPrimitiveType_TRIANGLE
        else {
            std::cerr<<"ignoring mesh type: "<<mesh->mPrimitiveTypes<<std::endl;
        }
    }

    // recursively for each child node
    for(uint iChild(0); iChild<node->mNumChildren; iChild++) {
        const aiNode* const child = node->mChildren[iChild];
        getMesh(scene, child, obj_mesh, transforms);
    }
}

template<typename T>
size_t vec_byte_size(const typename std::vector<T>& vec) {
    return sizeof(T) * vec.size();
}

MeshPtr getMesh(const std::string &path) {
    Assimp::Importer importer;

    const uint flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices;
//    const uint flags = aiProcess_Triangulate;
//    const uint flags = aiProcess_Triangulate | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;
//    const uint flags = 0;

//    std::string directory = path.substr(0, path.find_last_of('/'));

    //std::cout<<"directory: "<<directory<<std::endl;

    const aiScene* scene = importer.ReadFile(path, flags);

    if(!scene) {
        std::cerr<<"Import error: "<<importer.GetErrorString()<<std::endl;
    }

    //std::cout<<"material?: "<<scene->HasMaterials()<<std::endl;
    //std::cout<<"texture?: "<<scene->HasTextures()<<std::endl;

    if(scene->mNumMaterials>1) {
        std::cerr<<"more than 1 material is currently not supported!"<<std::endl;
    }

    std::vector<aiMatrix4x4> transforms;

    MeshPtr mesh(new Mesh());
    (*mesh).directory = path.substr(0, path.find_last_of('/'));

    getMesh(scene, scene->mRootNode, (*mesh), transforms);

    //Mesh mesh(verts.size(), faces.size());
//    memcpy(mesh.vertices.data(), verts.data(), vec_byte_size(verts));
//    memcpy(mesh.normals.data(), normals.data(), vec_byte_size(normals));
//    memcpy(mesh.faces.data(), faces.data(), vec_byte_size(faces));

    const uint nMeshes = scene->mNumMeshes;

    //std::cout<<"meshes: "<<nMeshes<<std::endl;

//    std::vector<MeshPtr> meshes;

//    for(uint iMesh(0); iMesh<nMeshes; iMesh++) {
//        const aiMesh *aimesh = scene->mMeshes[iMesh];

//        std::cout<<"name: "<<aimesh->mName.C_Str()<<std::endl;

//        std::cout<<"colour?: "<<aimesh->GetNumColorChannels()<<std::endl;
//        std::cout<<"UV?: "<<aimesh->GetNumUVChannels()<<std::endl;

//        if(scene->HasMaterials()) {
//            const aiMaterial *material = scene->mMaterials[aimesh->mMaterialIndex];

//            for(uint iText = 0; iText < material->GetTextureCount(aiTextureType_DIFFUSE); iText++) {
//                aiString path;
//                material->GetTexture(aiTextureType_DIFFUSE, iText, &path);
//                std::cout<<"diffuse texture at: "<<path.C_Str()<<std::endl;
//            } // texture
//        } // materials


//        const uint nVerts = aimesh->mNumVertices;
//        const uint nFaces = aimesh->mNumFaces;

//        Mesh mesh(nVerts, nFaces);

//        std::cout<<"verts: "<<nVerts<<std::endl;
//        std::cout<<"faces: "<<nFaces<<std::endl;

//        for(uint i = 0; i<nVerts; i++) {
//            // vertices
//            mesh.vertices.row(i) << aimesh->mVertices[i].x, aimesh->mVertices[i].y, aimesh->mVertices[i].z;

//            // normals
//            if(aimesh->HasNormals())
//                mesh.normals.row(i) << aimesh->mNormals[i].x, aimesh->mNormals[i].y, aimesh->mNormals[i].z;

//            // texture coordiantes
//            if(aimesh->mTextureCoords[0]!=NULL)
//                mesh.texture.row(i) << aimesh->mTextureCoords[0][i].x, aimesh->mTextureCoords[0][i].y;

//            // colour, r, g, b
//            if(aimesh->mColors[0]!=NULL)
//                mesh.colour.row(i) << aimesh->mColors[0][i].r, aimesh->mColors[0][i].g, aimesh->mColors[0][i].b;

//            // TODO: material
//        }

//        for(uint i = 0; i<nFaces; i++) {
//            const aiFace face = aimesh->mFaces[i];

//            // only accept triangulated meshes
//            assert(face.mNumIndices==3);

//            mesh.faces.row(i) << face.mIndices[0], face.mIndices[1], face.mIndices[2];
//        }

//        meshes.push_back(std::make_shared<Mesh>(mesh));
//    } // for all nMeshes

    //return std::make_shared<Mesh>(mesh);
//    return std::move<Mesh>(mesh);
    return std::move(mesh);
}

} // namespace MeshLoader
