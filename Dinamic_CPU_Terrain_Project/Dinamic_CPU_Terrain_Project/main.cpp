#include "lib.h"
#include "strutture.h"
#include "utilities.h"
#include "guiHandler.h"
#include "modelLoader.h"
#include "noiseHandler.h"
#include "shaderHandler.h"
#include "cameraHandler.h"
#include "bufferHandler.h"
#include "textureHandler.h"
#include "geometryHandler.h"
#include "interactionHandler.h"

//GLuint primitivesGenerated = 0; //Numero di primitive generate

int height = 600; //Altezza della finestra
int width = 600; //Larghezza della finestra

float Theta = -90.0f; //Angolo per la rotazione orizzontale
float Phi = 0.0f; //Angolo per la rotazione verticale
float moveSpeed = 0.002;
long long startTimeMillis = 0;

float terrainSize = 20.0f;

bool mouseLocked = true;
bool lineMode = true;
bool mainCharacter = true;

ViewSetup SetupTelecamera;
PerspectiveSetup SetupProspettiva;

pointLight light;

extern vector<unsigned int> indices;
extern vector<BoneInfo> bone_info_walking;
extern vector<BoneInfo> bone_info_standing;
extern const aiScene* scene_walking;
extern const aiScene* scene_standing;

int main() {
    int division = 12;
    int numSpheres = 100;
    int numOctaves = 8;
    int terrainTextureSize = 512;
    float offset = 5.0f;
    float r_min = 0.01f, r_max = 0.05f;

    mat4 model = mat4(1.0f); //(Nessuna trasformazione) --> Qui potrei scalare, ruotare o traslare 
    mat4 view = lookAt(vec3(0.0f, 0.0f, 2.0f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f)); //(Davanti all'origine)
    mat4 proj = perspective(radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f); //(FOV: 45, ASPECT: 4.3, ZNEAR: 0.1, ZFAR: 100)
    
    //GLFW
    glfwInit(); //Inizializzazione di GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4); //Specifica a GLFW che verrà utilizzato OpenGL versione 4.x (specifica la versione maggiore)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6); //Specifica a GLFW che verrà utilizzato OpenGL versione 4.6 (specifica la versione minore)
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Richiede un core profile di OpenGL (che esclude le funzionalità deprecate)

    GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor(); // Prendi il monitor principale
    const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor); // Prendi le modalità video del monitor (risoluzione, refresh rate, ecc)
    height = mode->height;
    width = mode->width;
    GLFWwindow* window = glfwCreateWindow(width, height, "Tessellation Shader", primaryMonitor, nullptr); // Crea la finestra fullscreen con le dimensioni del monitor
    //GLFWwindow* window = glfwCreateWindow(width, height, "Tessellation Shader", nullptr, nullptr);

    if (!window) { //Gestione dell'errore
        std::cerr << "Errore nella creazione della finestra GLFW\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window); //Attiva il contesto OpenGL associato alla finestra creata, rendendo il contesto corrente per il thread in cui viene chiamata


    //CALLBACKS
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    

    //Inizializzazione di GLAD (carica i puntatori alle funzioni OpenGL)
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Errore nell'inizializzazione di GLAD\n";
        return -1;
    }


    //ILLUMINAZIONE
    light.position = { 10.0, 50.0, -10.0 };
    light.color = { 1.0,1.0,1.0 };
    light.power = 3.0f;


    //TEXTURES
    vector<float> fbmData = generateFBMData(terrainTextureSize, terrainTextureSize, numOctaves);
    GLuint fbmTexture = createFloatTexture2D(terrainTextureSize, terrainTextureSize, fbmData);
    vector<GLuint> allTextures = loadAllTextures();
    GLuint modelTexture = loadSingleTexture("Model/Knight/textures/texture_embedded_0.png");
    GLuint skyboxTexture = loadSkybox();


    //MAPPA, FOGLIE e ALGHE
    vector<Vertex> baseVertices;
    vector<unsigned int> baseIndices;
    generateTerrainBase(division, terrainSize, baseVertices);
    vector<Patch> patchesBase = generatePatches(baseVertices, division);
    float minDist = 1.5f, maxDist = 2.5f, minSubdiv = 6, maxSubdiv = 32;
    unsigned int baseIndex = 0;
    vector<Vertex> grassVertices;
    vector<unsigned int> grassIndices;
    vector<Vertex> kelpVertices;
    vector<unsigned int> kelpIndices;
    const float GRASS_MIN_TERRAIN_HEIGHT = 0.1;
    const float GRASS_MAX_TERRAIN_HEIGHT = 0.6;
    const float KELP_MAX_TERRAIN_HEIGHT = -0.6;


    //SFERE
    vector<Vertex> sphereVertices;
    vector<unsigned int> sphereIndices;
    unsigned int vertexOffset = 0;
    vector<vec3> centers;
    vector<float> rads;
    for (int i = 0; i < numSpheres; i++) {
        centers.push_back(randomPosition(terrainSize));
        rads.push_back(randomFloat(r_min, r_max));
    }
    

    //MODEL
    //Texture
    string path = "Model/Knight/source/castle_guard_01.fbx";
    //extractEmbeddedTextures(path, "Model/Knight/textures");
    //Walking
    path = "Model/Knight/source/Walking.fbx";
    loadModel(path, WALKING);
    ModelBufferPair walkingModelPair = INIT_MODEL_BUFFERS();
    //Standing
    path = "Model/Knight/source/Standing.fbx";
    loadModel(path, STANDING);
    ModelBufferPair standingModelPair = INIT_MODEL_BUFFERS();


    //SKYBOX
    vector<float> skyboxVertices = generateSkyboxCube();
    BufferPair skyboxPair = INIT_SIMPLE_VERTEX_BUFFERS(skyboxVertices);


    //SHADER PROGRAMS
    unsigned int terrainProgram = createSimpleShaderProgram(
        "vertex.glsl",
        "fragment.glsl"
    ); 
    unsigned int leavesProgram = createSimpleShaderProgram(
        "vertex_leaves.glsl",
        "fragment_leaves.glsl"
    );
    unsigned int kelpsProgram = createSimpleShaderProgram(
        "vertex_kelps.glsl",
        "fragment_kelps.glsl"
    );
    unsigned int sphereProgram = createSimpleShaderProgram(
        "vertex_sphere.glsl", 
        "fragment_sphere.glsl"
    );
    unsigned int modelProgram = createSimpleShaderProgram(
        "vertex_model.glsl",
        "fragment_model.glsl"
    );
    unsigned int skyboxProgram = createSimpleShaderProgram(
        "vertex_skybox.glsl",
        "fragment_skybox.glsl"
    );


    //UNIFORMS
    //Terrain program
    int modelLocation = glGetUniformLocation(terrainProgram, "model");
    int viewLocation = glGetUniformLocation(terrainProgram, "view");
    int projLocation = glGetUniformLocation(terrainProgram, "proj");
    int cameraPosLocTerrain = glGetUniformLocation(terrainProgram, "ViewPos");
    int lightPosLocTerrain = glGetUniformLocation(terrainProgram, "light.position");
    int lightColorLocTerrain = glGetUniformLocation(terrainProgram, "light.color");
    int lightPowerLocTerrain = glGetUniformLocation(terrainProgram, "light.power");
    //Leaves program
    int modelLocation_leaves = glGetUniformLocation(leavesProgram, "model");
    int viewLocation_leaves = glGetUniformLocation(leavesProgram, "view");
    int projLocation_leaves = glGetUniformLocation(leavesProgram, "proj");
    int cameraPos_leaves = glGetUniformLocation(leavesProgram, "ViewPos");
    int lightPos_leaves = glGetUniformLocation(leavesProgram, "lightPos");
    //Leaves program
    int modelLocation_kelps = glGetUniformLocation(kelpsProgram, "model");
    int viewLocation_kelps = glGetUniformLocation(kelpsProgram, "view");
    int projLocation_kelps = glGetUniformLocation(kelpsProgram, "proj");
    int cameraPos_kelps = glGetUniformLocation(kelpsProgram, "ViewPos");
    int lightPos_kelps = glGetUniformLocation(kelpsProgram, "lightPos");
    //Sphere program
    int modelLocation_sphere = glGetUniformLocation(sphereProgram, "model");
    int viewLocation_sphere = glGetUniformLocation(sphereProgram, "view");
    int projLocation_sphere = glGetUniformLocation(sphereProgram, "proj");
    int cameraPos_sphere = glGetUniformLocation(sphereProgram, "ViewPos");
    int lightPos_sphere = glGetUniformLocation(sphereProgram, "light.position");
    int lightColor_sphere = glGetUniformLocation(sphereProgram, "light.color");
    int lightPower_sphere = glGetUniformLocation(sphereProgram, "light.power");
    //Model program
    int modelLoc = glGetUniformLocation(modelProgram, "model");
    int viewLoc = glGetUniformLocation(modelProgram, "view");
    int projLoc = glGetUniformLocation(modelProgram, "proj");
    int cameraPosLoc = glGetUniformLocation(modelProgram, "ViewPos");
    int lightPosLoc = glGetUniformLocation(modelProgram, "light.position");
    int lightColorLoc = glGetUniformLocation(modelProgram, "light.color");
    int lightPowerLoc = glGetUniformLocation(modelProgram, "light.power");
    GLuint bonesLoc = glGetUniformLocation(modelProgram, "bones");
    //Skybox program
    int viewLocSkybox = glGetUniformLocation(skyboxProgram, "View");
    int projLocSkybox = glGetUniformLocation(skyboxProgram, "Projection");


    //SETTINGS
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //Imposta la modalità Wireframe per vedere le suddivisioni fatte dallo shader
    glDisable(GL_CULL_FACE); //Disabilita il culling
    glEnable(GL_DEPTH_TEST); //Abilita il depth test


    //TELECAMERA
    INIT_CAMERA_PROJECTION();


    //GUI
    initializeGui(window); //Inizializza la finestra di interazione


    //TIME
    startTimeMillis = static_cast<long long>(glfwGetTime() * 1000.0);


    //MODEL MOVEMENT
    vec3 modelMovement = vec3(0.0f);
    vec3 previousModelMovement = vec3(0.0f);
    vec3 modelWorldPos = vec3(0.0f); //posizione assoluta del modello in world space
    mat4 tiltMatrix = mat4(1.0f);
    float rotationAngle = 0.0f;


    //MAIN LOOP
    while (!glfwWindowShouldClose(window)) {

        //SETTINGS
        long long currentTimeMillis = static_cast<long long>(glfwGetTime() * 1000.0);
        float animationTimeSec = ((float)(currentTimeMillis - startTimeMillis)) / 1000.0f;

        if (lineMode) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //Imposta la modalità Wireframe per vedere le suddivisioni fatte dallo shader
        }
        else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); //Imposta la modalità Fill per vedere le suddivisioni riempite
        }

        float timeValue = glfwGetTime(); //Restituisce il tempo in secondi dall'avvio

        glPatchParameteri(GL_PATCH_VERTICES, 3); //Dice a OpenGL che ogni patch ha 3 vertici

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // Imposta il colore di clear (puoi cambiarlo)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        //TERRAIN DATA
        baseVertices.clear();
        baseIndices.clear();
        unsigned int terrainBaseIndex = 0;
        unsigned int grassBaseIndex = 0;
        unsigned int kelpBaseIndex = 0;

        vec3 lodReferencePos = mainCharacter ? modelWorldPos : SetupTelecamera.position;
        for (const Patch& patch : patchesBase) {
            // Calcolo centri di ciascun lato
            vec3 centerTop = (patch.v0 + patch.v1) * 0.5f;
            vec3 centerRight = (patch.v1 + patch.v2) * 0.5f;
            vec3 centerBottom = (patch.v2 + patch.v3) * 0.5f;
            vec3 centerLeft = (patch.v3 + patch.v0) * 0.5f;

            // Calcolo distanze da riferimento
            float distTop = length(centerTop - lodReferencePos);
            float distRight = length(centerRight - lodReferencePos);
            float distBottom = length(centerBottom - lodReferencePos);
            float distLeft = length(centerLeft - lodReferencePos);

            // Determino subdiv per ciascun lato
            int subdivTop = lodFromDistance(distTop, minDist, maxDist, minSubdiv, maxSubdiv);
            int subdivRight = lodFromDistance(distRight, minDist, maxDist, minSubdiv, maxSubdiv);
            int subdivBottom = lodFromDistance(distBottom, minDist, maxDist, minSubdiv, maxSubdiv);
            int subdivLeft = lodFromDistance(distLeft, minDist, maxDist, minSubdiv, maxSubdiv);

            // Salvo indice di partenza per estrarre i vertici appena generati
            size_t startIndex = baseIndices.size();

            // Genero mesh della patch con LOD per lato
            generatePatchMesh(patch, subdivTop, subdivRight, subdivBottom, subdivLeft,
                fbmData, terrainTextureSize, terrainTextureSize, baseVertices, baseIndices, terrainBaseIndex);

            // Calcolo quanti vertici sono stati aggiunti dalla patch
            size_t endIndex = baseIndices.size();

            // Ora creiamo erba/alghe per ogni triangolo aggiunto
            for (size_t i = startIndex; i < endIndex; i += 3) {
                Vertex v0 = baseVertices[baseIndices[i]];
                Vertex v1 = baseVertices[baseIndices[i + 1]];
                Vertex v2 = baseVertices[baseIndices[i + 2]];

                vec3 normal = normalize(cross(v1.pos - v0.pos, v2.pos - v0.pos));
                if (normal.y < 0) normal = -normal;

                std::vector<vec3> positions = {
                    v0.pos,
                    v1.pos,
                    v2.pos,
                    (v0.pos + v1.pos) * 0.5f,
                    (v1.pos + v2.pos) * 0.5f,
                    (v2.pos + v0.pos) * 0.5f,
                    (v0.pos + v1.pos + v2.pos) / 3.0f,
                    (v0.pos + v1.pos + v2.pos) / 3.0f  // raddoppiato come nel GS
                };

                for (const auto& pos : positions) {
                    if (length(pos - lodReferencePos) > minDist) continue;

                    if (pos.y > GRASS_MIN_TERRAIN_HEIGHT && pos.y < GRASS_MAX_TERRAIN_HEIGHT) {
                        generateGrassBlade(pos, normal, grassVertices, grassIndices, grassBaseIndex);
                    }
                    else if (pos.y < KELP_MAX_TERRAIN_HEIGHT) {
                        generateKelp(pos, kelpVertices, kelpIndices, kelpBaseIndex);
                    }
                }
            }
        }

        MeshBuffers terrainBuffers = INIT_VERTEX_BUFFERS(baseVertices, baseIndices);
        MeshBuffers grassBuffers = INIT_VERTEX_BUFFERS(grassVertices, grassIndices);
        MeshBuffers kelpBuffers = INIT_VERTEX_BUFFERS(kelpVertices, kelpIndices);



        //SFERE
        sphereVertices.clear();
        sphereIndices.clear();
        vertexOffset = 0;
        for (int i = 0; i < numSpheres; i++) {
            vec3 center = centers[i];
            float radius = rads[i];

            // Calcola la distanza dal riferimento LOD (camera o altro)
            float dist = length(center - lodReferencePos);

            // Determina il numero di segmenti in base alla distanza
            int lodSegments;
            float minDistStars = 5.0f;
            float maxDistStars = 10.0f;
            float t = clamp((dist - minDistStars) / (maxDistStars - minDistStars), 0.0f, 1.0f);
            lodSegments = (int)mix(32, 8, t);

            // Genera i vertici della sfera con lat/lon segmentati
            vector<Vertex> verts = generateSphereCPU(center, radius, lodSegments, lodSegments);

            // Genera indici per una sfera "standard"
            vector<unsigned int> inds = generateSphereIndices(lodSegments, lodSegments);

            // Aggiungi i vertici, con normale e centro
            for (const Vertex& v : verts) {
                // Assumo Vertex con pos, normal e center
                sphereVertices.push_back(Vertex{ v.pos, v.normal, center });
            }

            // Aggiusta indici rispetto all'offset dei vertici già presenti
            for (unsigned int idx : inds) {
                sphereIndices.push_back(vertexOffset + idx);
            }

            vertexOffset += (unsigned int)verts.size();
        }
        MeshBuffers sphereBuffers = INIT_VERTEX_BUFFERS(sphereVertices, sphereIndices);




        //SPHERE PROGRAM
        glUseProgram(sphereProgram);

        glUniformMatrix4fv(modelLocation_sphere, 1, GL_FALSE, value_ptr(model));
        glUniformMatrix4fv(viewLocation_sphere, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLocation_sphere, 1, GL_FALSE, value_ptr(proj));
        glUniform3fv(cameraPos_sphere, 1, value_ptr(SetupTelecamera.position));
        glUniform3fv(lightPos_sphere, 1, value_ptr(light.position));
        glUniform3fv(lightColor_sphere, 1, value_ptr(light.color));
        glUniform1f(lightPower_sphere, light.power);

        glBindVertexArray(sphereBuffers.vao);
        glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0);
        

        //MODEL PROGRAM
        glUseProgram(modelProgram);

        glActiveTexture(GL_TEXTURE0 + 0);
        glBindTexture(GL_TEXTURE_2D, modelTexture);
        string uniformName = "modelTexture";
        GLint location = glGetUniformLocation(modelProgram, uniformName.c_str());
        glUniform1i(location, 0);

        ModelState state;
        bool isMoving = length(modelMovement - previousModelMovement) > 0.00001f;
        state = isMoving ? WALKING : STANDING;

        //aggiornamento dell'animazione del personaggio (se presente)
        if (state == WALKING) {
            if (scene_walking && scene_walking->mNumAnimations > 0 && scene_walking->mAnimations[0]) {
                float ticksPerSecond = scene_walking->mAnimations[0]->mTicksPerSecond != 0 ? scene_walking->mAnimations[0]->mTicksPerSecond : 25.0f; //quanti tick al secondo
                float timeInTicks = animationTimeSec * ticksPerSecond; //quanti tick sono passati
                float animationTimeTicks = fmod(timeInTicks, scene_walking->mAnimations[0]->mDuration); //prendo la parte decimale dell'operazione modulo (animazione continua)
                updateBoneTransforms(animationTimeTicks, state);
            }
        }
        else {
            if (scene_standing && scene_standing->mNumAnimations > 0 && scene_standing->mAnimations[0]) {
                float ticksPerSecond = scene_standing->mAnimations[0]->mTicksPerSecond != 0 ? scene_standing->mAnimations[0]->mTicksPerSecond : 25.0f; //quanti tick al secondo
                float timeInTicks = animationTimeSec * ticksPerSecond; //quanti tick sono passati
                float animationTimeTicks = fmod(timeInTicks, scene_standing->mAnimations[0]->mDuration); //prendo la parte decimale dell'operazione modulo (animazione continua)
                updateBoneTransforms(animationTimeTicks, state);
            }
        }

        mat4 objectModel = mat4(1.0f);
        objectModel = translate(objectModel, modelWorldPos);
        objectModel *= tiltMatrix;
        objectModel = scale(objectModel, vec3(0.0005f));
        objectModel = rotate(objectModel, radians(float(180)), vec3(0.0f, 1.0f, 0.0f));
        objectModel = rotate(objectModel, radians(rotationAngle), vec3(0.0f, 1.0f, 0.0f));
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, value_ptr(objectModel));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, value_ptr(proj));
        glUniform3fv(cameraPosLoc, 1, value_ptr(SetupTelecamera.position));
        glUniform3fv(lightPosLoc, 1, value_ptr(light.position));
        glUniform3fv(lightColorLoc, 1, value_ptr(light.color));
        glUniform1f(lightPowerLoc, light.power);

        mat4 boneTransforms[128];

        if (state == WALKING) {
            for (int i = 0; i < bone_info_walking.size(); i++)
                boneTransforms[i] = bone_info_walking[i].finalTransform;

            glUniformMatrix4fv(bonesLoc, bone_info_walking.size(), GL_FALSE, value_ptr(boneTransforms[0]));
        }
        else {
            for (int i = 0; i < bone_info_standing.size(); i++)
                boneTransforms[i] = bone_info_standing[i].finalTransform;

            glUniformMatrix4fv(bonesLoc, bone_info_standing.size(), GL_FALSE, value_ptr(boneTransforms[0]));
        }


        glBindVertexArray(walkingModelPair.vao);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);


        //TERRAIN PROGRAM
        glUseProgram(terrainProgram);

        for (int i = 0; i < allTextures.size(); i++) { //Attiva le texture
            glActiveTexture(GL_TEXTURE0 + i);
            glBindTexture(GL_TEXTURE_2D, allTextures[i]);
        }
        //Lega le texture alle relative variabili uniform
        for (int i = 0; i < allTextures.size(); i++) {
            string uniformName = "texture" + std::to_string(i);
            GLint location = glGetUniformLocation(terrainProgram, uniformName.c_str());
            glUniform1i(location, i);
        }

        glUniformMatrix4fv(modelLocation, 1, GL_FALSE, value_ptr(model));
        glUniformMatrix4fv(viewLocation, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLocation, 1, GL_FALSE, value_ptr(proj));
        glUniform3fv(cameraPosLocTerrain, 1, value_ptr(SetupTelecamera.position));
        glUniform3fv(lightPosLocTerrain, 1, value_ptr(light.position));
        glUniform3fv(lightColorLocTerrain, 1, value_ptr(light.color));
        glUniform1f(lightPowerLocTerrain, light.power);
        
        glBindVertexArray(terrainBuffers.vao);
        glDrawElements(GL_TRIANGLES, baseIndices.size(), GL_UNSIGNED_INT, 0);


        //LEAVES PROGRAM
        glUseProgram(leavesProgram);

        glUniformMatrix4fv(modelLocation_leaves, 1, GL_FALSE, value_ptr(model));
        glUniformMatrix4fv(viewLocation_leaves, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLocation_leaves, 1, GL_FALSE, value_ptr(proj));
        glUniform3fv(cameraPos_leaves, 1, value_ptr(SetupTelecamera.position));
        glUniform3fv(lightPos_leaves, 1, value_ptr(light.position));
        
        glBindVertexArray(grassBuffers.vao);
        glDrawElements(GL_TRIANGLES, grassIndices.size(), GL_UNSIGNED_INT, 0);


        //KELPS PROGRAM
        glUseProgram(kelpsProgram);

        glUniformMatrix4fv(modelLocation_kelps, 1, GL_FALSE, value_ptr(model));
        glUniformMatrix4fv(viewLocation_kelps, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLocation_kelps, 1, GL_FALSE, value_ptr(proj));
        glUniform3fv(cameraPos_kelps, 1, value_ptr(SetupTelecamera.position));
        glUniform3fv(lightPos_kelps, 1, value_ptr(light.position));

        glBindVertexArray(kelpBuffers.vao);
        glDrawElements(GL_TRIANGLES, kelpIndices.size(), GL_UNSIGNED_INT, 0);


        //SKYBOX
        glDepthFunc(GL_LEQUAL);       // per permettere la skybox in fondo
        glDepthMask(GL_FALSE);        // disattiva scrittura nello z-buffer

        glUseProgram(skyboxProgram);
        
        glUniform1i(glGetUniformLocation(skyboxProgram, "skybox"), 0);
        glUniformMatrix4fv(viewLocSkybox, 1, GL_FALSE, value_ptr(view));
        glUniformMatrix4fv(projLocSkybox, 1, GL_FALSE, value_ptr(proj));

        glBindVertexArray(skyboxPair.vao);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture);

        glDrawArrays(GL_TRIANGLES, 0, 36);
        
        glBindVertexArray(0);
        glDepthMask(GL_TRUE);         // riattiva scrittura per gli oggetti normali
        glDepthFunc(GL_LESS);         // ripristina depth test standard


        renderGui();
        glfwSwapBuffers(window); //Scambia il buffer frontale con quello posteriore
        glfwPollEvents(); //Controlla e gestisce gli eventi della finestra (input tastiera, mouse, ...)


        //MATRICI DI TRASFORMAZIONE
        view = lookAt(SetupTelecamera.position, SetupTelecamera.target, SetupTelecamera.upVector);
        proj = perspective(radians(SetupProspettiva.fovY), SetupProspettiva.aspect, SetupProspettiva.near_plane, SetupProspettiva.far_plane);
    
        auto inputResult = process_input(window);
        previousModelMovement = modelMovement;
        if (length(inputResult.first) > 0.0001f) {
            modelMovement += inputResult.first;
            modelWorldPos += inputResult.first;
            rotationAngle = inputResult.second;
        }
    }


    //TERMINAZIONE
    glDeleteProgram(terrainProgram);
    glDeleteProgram(sphereProgram);
    glDeleteProgram(leavesProgram);
    glDeleteProgram(kelpsProgram);
    glDeleteProgram(modelProgram);
    destroyGui();
    glfwDestroyWindow(window); //Elimina la finestra GLFW
    glfwTerminate(); //Termina la libreria GLFW, liberando tutte le risorse rimaste
    return 0;
}