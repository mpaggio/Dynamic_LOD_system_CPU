#include "modelLoader.h"
#include "textureHandler.h"

mat4 globalInverseTransformation;

vector<vec3> positions;
vector<vec3> normals;
vector<vec2> texCoords;
vector<unsigned int> indices;

vector<BoneInfo> bone_info_walking; //contiene offset matrix e la trasformazione animata finale (walking)
vector<BoneInfo> bone_info_standing; //contiene offset e trasformazione animata (standing)

vector<VertexBoneData> vertices_to_bones; //mapping dai vertici alle informazioni delle ossa che li influenzano (mapping inverso)
vector<int> mesh_vertices; //contiene l'indice iniziale di ogni mesh all'interno dell'array globale dei vertici
map<string, unsigned int> bone_name_to_index_walking; //mapping dai nomi delle ossa agli indici relativi (walking)
map<string, unsigned int> bone_name_to_index_standing; //(standing)

Importer importerBindPose;
Importer importerWalking;
Importer importerStanding;
const aiScene* scene_bind_pose;
const aiScene* scene_walking;
const aiScene* scene_standing;


void printMat4(const glm::mat4& mat) {
    for (int i = 0; i < 4; ++i)
        cout << mat[i][0] << " " << mat[i][1] << " " << mat[i][2] << " " << mat[i][3] << endl;
}

mat4 aiMatrix4x4_to_mat4(const aiMatrix4x4& from) {
    mat4 to;
    
    to[0][0] = from.a1; to[1][0] = from.a2; to[2][0] = from.a3; to[3][0] = from.a4;
    to[0][1] = from.b1; to[1][1] = from.b2; to[2][1] = from.b3; to[3][1] = from.b4;
    to[0][2] = from.c1; to[1][2] = from.c2; to[2][2] = from.c3; to[3][2] = from.c4;
    to[0][3] = from.d1; to[1][3] = from.d2; to[2][3] = from.d3; to[3][3] = from.d4;
    
    return to;
}

aiNodeAnim* findNodeAnim(const aiAnimation* sceneAnimation, const string nodeName) {
    for (int i = 0; i < sceneAnimation->mNumChannels; i++) {
        aiNodeAnim* nodeAnim = sceneAnimation->mChannels[i];

        if (string(nodeAnim->mNodeName.data) == nodeName) {
            return nodeAnim;
        }
    }
    return nullptr;
}

unsigned int findScaling(float animationTimeTicks, const aiNodeAnim* nodeAnimation) {
    //Restituisce l'indice dell'animazione corrente in base al tempo
    for (unsigned int i = 0; i < nodeAnimation->mNumScalingKeys - 1; i++) {
        float t = (float)nodeAnimation->mScalingKeys[i + 1].mTime;

        if (animationTimeTicks < t) {
            return i;
        }
    }

    return 0;
}

unsigned int findTranslation(float animationTimeTicks, const aiNodeAnim* nodeAnimation) {
    //Restituisce l'indice dell'animazione corrente in base al tempo
    for (unsigned int i = 0; i < nodeAnimation->mNumPositionKeys - 1; i++) {
        float t = (float)nodeAnimation->mPositionKeys[i + 1].mTime;

        if (animationTimeTicks < t) {
            return i;
        }
    }

    return 0;
}

unsigned int findRotation(float animationTimeTicks, const aiNodeAnim* nodeAnimation) {
    //Restituisce l'indice dell'animazione corrente in base al tempo
    for (unsigned int i = 0; i < nodeAnimation->mNumRotationKeys - 1; i++) {
        float t = (float)nodeAnimation->mRotationKeys[i + 1].mTime;

        if (animationTimeTicks < t) {
            return i;
        }
    }

    return 0;
}

void CalcInterpolatedScaling(aiVector3D& scaling, float animationTimeTickets, const aiNodeAnim* nodeAnimation) {
    // se è presente una sola key non c'è bisogno di interpolare
    if (nodeAnimation->mNumScalingKeys == 1) {
        scaling = nodeAnimation->mScalingKeys[0].mValue;
        return;
    }

    unsigned int scalingIndex = findScaling(animationTimeTickets, nodeAnimation);
    unsigned int nextScalingIndex = scalingIndex + 1;

    float t1 = (float)nodeAnimation->mScalingKeys[scalingIndex].mTime;
    float t2 = (float)nodeAnimation->mScalingKeys[nextScalingIndex].mTime;
    float deltaTime = t2 - t1;
    float factor = (animationTimeTickets - (float)t1) / deltaTime;

    const aiVector3D& start = nodeAnimation->mScalingKeys[scalingIndex].mValue;
    const aiVector3D& end = nodeAnimation->mScalingKeys[nextScalingIndex].mValue;
    aiVector3D delta = end - start;
    
    scaling = start + factor * delta;
}

void CalcInterpolatedTranslation(aiVector3D& translation, float animationTimeTickets, const aiNodeAnim* nodeAnimation) {
    // se è presente una sola key non c'è bisogno di interpolare
    if (nodeAnimation->mNumPositionKeys == 1) {
        translation = nodeAnimation->mPositionKeys[0].mValue;
        return;
    }

    unsigned int translationIndex = findTranslation(animationTimeTickets, nodeAnimation);
    unsigned int nextTranslationIndex = translationIndex + 1;

    float t1 = (float)nodeAnimation->mPositionKeys[translationIndex].mTime;
    float t2 = (float)nodeAnimation->mPositionKeys[nextTranslationIndex].mTime;
    float deltaTime = t2 - t1;
    float factor = (animationTimeTickets - (float)t1) / deltaTime;

    const aiVector3D& start = nodeAnimation->mPositionKeys[translationIndex].mValue;
    const aiVector3D& end = nodeAnimation->mPositionKeys[nextTranslationIndex].mValue;
    aiVector3D delta = end - start;

    translation = start + factor * delta;
}

void CalcInterpolatedRotation(aiQuaternion& rotation, float animationTimeTickets, const aiNodeAnim* nodeAnimation) {
    // se è presente una sola key non c'è bisogno di interpolare
    if (nodeAnimation->mNumRotationKeys == 1) {
        rotation = nodeAnimation->mRotationKeys[0].mValue;
        return;
    }

    unsigned int rotationIndex = findRotation(animationTimeTickets, nodeAnimation);
    unsigned int nextRotationIndex = rotationIndex + 1;

    float t1 = (float)nodeAnimation->mRotationKeys[rotationIndex].mTime;
    float t2 = (float)nodeAnimation->mRotationKeys[nextRotationIndex].mTime;
    float deltaTime = t2 - t1;
    float factor = (animationTimeTickets - (float)t1) / deltaTime;

    const aiQuaternion& start = nodeAnimation->mRotationKeys[rotationIndex].mValue;
    const aiQuaternion& end = nodeAnimation->mRotationKeys[nextRotationIndex].mValue;
    aiQuaternion::Interpolate(rotation, start, end, factor);

    rotation.Normalize();
}

void readNodeHierarchy(float animationTimeTicks, const aiNode* node, const mat4& parentTransform, ModelState state) {
    string nodeName = node->mName.data;
    aiAnimation* animation;
    if (state == WALKING) {
        animation = scene_walking->mAnimations[0];
    }
    else{
        animation = scene_standing->mAnimations[0];
    }
    
    mat4 nodeTransformation = aiMatrix4x4_to_mat4(node->mTransformation);
    aiNodeAnim* nodeAnimation = findNodeAnim(animation, nodeName);

    if (nodeAnimation) {
        aiVector3D scaling;
        mat4 scalingMatrix = mat4(1.0f);
        CalcInterpolatedScaling(scaling, animationTimeTicks, nodeAnimation);
        scalingMatrix = scale(scalingMatrix, vec3(scaling.x, scaling.y, scaling.z));

        aiQuaternion rotation;
        CalcInterpolatedRotation(rotation, animationTimeTicks, nodeAnimation);
        quat glmRotation = quat(rotation.w, rotation.x, rotation.y, rotation.z);
        mat4 rotationMatrix = toMat4(glmRotation);

        aiVector3D translation;
        mat4 translationMatrix = mat4(1.0f);;
        CalcInterpolatedTranslation(translation, animationTimeTicks, nodeAnimation);
        translationMatrix = translate(translationMatrix, vec3(translation.x, translation.y, translation.z));

        nodeTransformation = translationMatrix * rotationMatrix * scalingMatrix;
    }

    mat4 globalTransform = parentTransform * nodeTransformation;
    
    if (state == WALKING) {
        if (bone_name_to_index_walking.find(nodeName) != bone_name_to_index_walking.end()) {
            int boneIndex = bone_name_to_index_walking[nodeName];
            bone_info_walking[boneIndex].finalTransform = globalInverseTransformation * globalTransform * bone_info_walking[boneIndex].offsetMatrix;
        }
    }
    else {
        if (bone_name_to_index_standing.find(nodeName) != bone_name_to_index_standing.end()) {
            int boneIndex = bone_name_to_index_standing[nodeName];
            bone_info_standing[boneIndex].finalTransform = globalInverseTransformation * globalTransform * bone_info_standing[boneIndex].offsetMatrix;
        }
    }
        

    // Ricorsione per i figli
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        readNodeHierarchy(animationTimeTicks, node->mChildren[i], globalTransform, state);
    }
}

void updateBoneTransforms(float animationTime, ModelState state) {
    mat4 identity = mat4(1.0f);
    if (state == WALKING) {
        readNodeHierarchy(animationTime, scene_walking->mRootNode, identity, state);
    }
    else {
        readNodeHierarchy(animationTime, scene_standing->mRootNode, identity, state);
    }
}

int getBoneID(const aiBone* bone, ModelState state) {
    int boneID = 0;
    string boneName = bone->mName.C_Str(); //recupero il nome dell'osso

    if (state == WALKING) {
        if (bone_name_to_index_walking.find(boneName) == bone_name_to_index_walking.end()) { //se il nome non è stato trovato
            boneID = bone_name_to_index_walking.size(); //indice della nuova posizione (in fondo)
            bone_name_to_index_walking[boneName] = boneID; //inserisco nella posizione data l'indice del nuovo osso

            // Assicurati che bone_info abbia spazio
            if (bone_info_walking.size() <= boneID)
                bone_info_walking.resize(boneID + 1);
            // Salva la offset matrix dell'osso
            bone_info_walking[boneID].offsetMatrix = aiMatrix4x4_to_mat4(bone->mOffsetMatrix);
            bone_info_walking[boneID].finalTransform = mat4(1.0f);
        }
        else { //se il nome è stato trovato
            boneID = bone_name_to_index_walking[boneName]; //prendo il suo indice
        }
    }
    else {
        if (bone_name_to_index_standing.find(boneName) == bone_name_to_index_standing.end()) { //se il nome non è stato trovato
            boneID = bone_name_to_index_standing.size(); //indice della nuova posizione (in fondo)
            bone_name_to_index_standing[boneName] = boneID; //inserisco nella posizione data l'indice del nuovo osso

            // Assicurati che bone_info abbia spazio
            if (bone_info_standing.size() <= boneID)
                bone_info_standing.resize(boneID + 1);
            // Salva la offset matrix dell'osso
            bone_info_standing[boneID].offsetMatrix = aiMatrix4x4_to_mat4(bone->mOffsetMatrix);
            bone_info_standing[boneID].finalTransform = mat4(1.0f);
        }
        else { //se il nome è stato trovato
            boneID = bone_name_to_index_standing[boneName]; //prendo il suo indice
        }
    }
    
    return boneID;
}

void loadMeshBones(const int meshIndex, const aiMesh* mesh, ModelState state) {
    for (int i = 0; i < mesh->mNumBones; i++) {
        const aiBone* bone = mesh->mBones[i];
        
        int boneID = getBoneID(bone, state); //recupero l'ID dell'osso

        for (int j = 0; j < bone->mNumWeights; j++) {
            const aiVertexWeight& vertexWeight = bone->mWeights[j];

            unsigned int globalVertexID = mesh_vertices[meshIndex] + vertexWeight.mVertexId; //calcolo l'indice del vertice nell'array globale

            vertices_to_bones[globalVertexID].addBone(boneID, vertexWeight.mWeight); //aggiungo le informazioni dell'osso ai dati del vertice
        }
    }
}

void loadSceneData(const aiScene* scene, ModelState state) {
    int total_vertices = 0;
    int total_indices = 0;
    int total_bones = 0;

    mesh_vertices.resize(scene->mNumMeshes); //inizializzo con il numero effettivo di mesh

    for (int i = 0; i < scene->mNumMeshes; i++) {
        const aiMesh* mesh = scene->mMeshes[i];
        total_vertices += mesh->mNumVertices;
        total_indices += mesh->mNumFaces * 3;
        total_bones += mesh->mNumBones;
    }

    positions.reserve(total_vertices);
    normals.reserve(total_vertices);
    texCoords.reserve(total_vertices);
    indices.reserve(total_indices);

    vertices_to_bones.resize(total_vertices); //inizializzo con il numero effettivo di vertici

    int vertex_offset = 0;
    for (int i = 0; i < scene->mNumMeshes; i++) {
        const aiMesh* mesh = scene->mMeshes[i];
        int num_vertices = mesh->mNumVertices;
        int num_indices = mesh->mNumFaces * 3;
        int num_bones = mesh->mNumBones;

        mesh_vertices[i] = vertex_offset; //assegno a ciascuna mesh l'indice di partenza corrispondente nell'array globale dei vertici

        // Vertici
        for (int v = 0; v < num_vertices; v++) {
            unsigned int globalVertexID = mesh_vertices[i] + v;
            VertexBoneData& vertex = vertices_to_bones[globalVertexID];

            // Position
            aiVector3D pos = mesh->mVertices[v];
            positions.push_back(vec3(pos.x, pos.y, pos.z));

            // Normal
            if (mesh->HasNormals()) {
                aiVector3D normal = mesh->mNormals[v];
                normals.push_back(vec3(normal.x, normal.y, normal.z));
            }
            else {
                aiVector3D backupNormal(0.0f, 1.0f, 0.0f);
                normals.push_back(vec3(backupNormal.x, backupNormal.y, backupNormal.z));
            }

            // Texture coords
            if (mesh->HasTextureCoords(0)) {
                aiVector3D uv = mesh->mTextureCoords[0][v];
                texCoords.push_back(vec2(uv.x, uv.y));
            }
            else {
                aiVector3D backupUV(0.0f, 0.0f, 0.0f);
                texCoords.push_back(vec2(backupUV.x, backupUV.y));
            }
        }

        // Indici
        for (int f = 0; f < mesh->mNumFaces; f++) {
            const aiFace& face = mesh->mFaces[f];
            indices.push_back(mesh_vertices[i] + face.mIndices[0]);
            indices.push_back(mesh_vertices[i] + face.mIndices[1]);
            indices.push_back(mesh_vertices[i] + face.mIndices[2]);
        }
    
        // Dati ossa
        if (mesh->HasBones()) {
            loadMeshBones(i, mesh, state);
        }

        vertex_offset += num_vertices;
    }

    for (auto& v : vertices_to_bones) {
        v.normalize();
    }
}

void loadModel(string modelPath, ModelState state) {
    aiMatrix4x4 transform;

    if (state == WALKING) {
        scene_walking = importerWalking.ReadFile(
            modelPath,
            aiProcess_Triangulate |
            aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices
        );

        if (!scene_walking || !scene_walking->HasMeshes()) {
            printf("Failed to load model or no meshes found\n");
            return;
        }

        if (!scene_walking->HasAnimations()) {
            printf("Model loaded, but no animations found\n");
        }

        transform = scene_walking->mRootNode->mTransformation;
        globalInverseTransformation = inverse(aiMatrix4x4_to_mat4(transform));

        loadSceneData(scene_walking, state);
    }
    else {
        scene_standing = importerStanding.ReadFile(
            modelPath,
            aiProcess_Triangulate |
            aiProcess_GenSmoothNormals |
            aiProcess_JoinIdenticalVertices
        );

        if (!scene_standing || !scene_standing->HasMeshes()) {
            printf("Failed to load model or no meshes found\n");
            return;
        }

        if (!scene_standing->HasAnimations()) {
            printf("Model loaded, but no animations found\n");
        }

        if (!scene_standing->HasTextures()) {
            printf("Model loaded, but no textures found\n");
        }

        transform = scene_standing->mRootNode->mTransformation;
        globalInverseTransformation = inverse(aiMatrix4x4_to_mat4(transform));

        loadSceneData(scene_standing, state);
    }
}

void extractEmbeddedTextures(const string modelPath, const string& outputDirectory) {
    scene_bind_pose = importerBindPose.ReadFile(
        modelPath,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_JoinIdenticalVertices
    );

    if (!scene_bind_pose->HasTextures()) {
        cout << "[INFO] Nessuna texture embedded trovata nel modello.\n";
        return;
    }

    for (unsigned int i = 0; i < scene_bind_pose->mNumTextures; ++i) {
        const aiTexture* texture = scene_bind_pose->mTextures[i];

        if (texture->mHeight == 0) {
            // Texture compressa (PNG, JPG, ecc.)
            string extension = texture->achFormatHint; // es: "png"
            string fileName = outputDirectory + "/texture_embedded_" + to_string(i) + "." + extension;

            ofstream fout(fileName, ios::binary);
            fout.write(reinterpret_cast<const char*>(texture->pcData), texture->mWidth);
            fout.close();

            cout << "[OK] Salvata texture embedded in: " << fileName << endl;
        }
        else {
            cout << "[WARN] Texture non compressa non gestita (RAW RGBA data)...\n";
        }
    }
}

vector<vec3> getModelBoundingVolume() {
    if (positions.empty())
        return {};

    vec3 m = positions[0];
    vec3 M = positions[0];

    // Calcola min e max su tutte le coordinate
    for (const auto& v : positions) {
        m.x = std::min(m.x, v.x);
        m.y = std::min(m.y, v.y);
        m.z = std::min(m.z, v.z);

        M.x = std::min(M.x, v.x);
        M.y = std::min(M.y, v.y);
        M.z = std::min(M.z, v.z);
    }

    // Crea e ritorna i 8 vertici del bounding box
    vector<vec3> boundingBoxVertices(8);
    boundingBoxVertices[0] = vec3(m.x, m.y, m.z);
    boundingBoxVertices[1] = vec3(M.x, m.y, m.z);
    boundingBoxVertices[2] = vec3(M.x, M.y, m.z);
    boundingBoxVertices[3] = vec3(m.x, M.y, m.z);
    boundingBoxVertices[4] = vec3(m.x, m.y, M.z);
    boundingBoxVertices[5] = vec3(M.x, m.y, M.z);
    boundingBoxVertices[6] = vec3(M.x, M.y, M.z);
    boundingBoxVertices[7] = vec3(m.x, M.y, M.z);

    return boundingBoxVertices;
}

vec3 getBoundingBoxBaseCenter() {
    if (positions.empty())
        return vec3(0.0f);

    vec3 m = positions[0];
    vec3 M = positions[0];

    for (const auto& v : positions) {
        m.x = std::min(m.x, v.x);
        m.y = std::min(m.y, v.y);
        m.z = std::min(m.z, v.z);

        M.x = std::max(M.x, v.x);
        M.y = std::max(M.y, v.y);
        M.z = std::max(M.z, v.z);
    }

    // Calcola centro base (media dei 4 vertici con z minima)
    vec3 baseCenter;
    baseCenter.x = (m.x + M.x) / 2.0f;
    baseCenter.y = (m.y + M.y) / 2.0f;
    baseCenter.z = m.z;

    return baseCenter;
}