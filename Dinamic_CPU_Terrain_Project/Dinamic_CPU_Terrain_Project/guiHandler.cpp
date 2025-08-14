#include "guiHandler.h"

extern bool lineMode;
extern bool mouseLocked;
extern bool mainCharacter;
extern float moveSpeed;

//extern GLuint primitivesGenerated;

extern pointLight light;

void initializeGui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
}

void renderGui() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    float currentFPS = ImGui::GetIO().Framerate;

    ImGui::SetNextWindowSize(ImVec2(400, 300));

    ImGui::Begin("Performance");

    ImGui::Text("Modalita' camera: %s", mouseLocked ? "ATTIVA" : "DISATTIVA");
    ImGui::Text("Premi tasto destro mouse per %s il mouse.", mouseLocked ? "sbloccare" : "bloccare");
    ImGui::Text("Premi tasto sinistro mouse per rallentare.");
    ImGui::Text("Premi \"L\" per entrare in modalita' LINE.");
    ImGui::Text("Premi \"F\" per entrare in modalita' FILL.");

    ImGui::Text("FPS: %.1f", currentFPS);
    //ImGui::Text("Primitive generate (GS): %d", primitivesGenerated);

    ImGui::Text("Current mode: %s", lineMode ? "LINE" : "FILL");

    ImGui::SliderFloat("position x", &light.position.x, -50.0f, 50.0f);
    ImGui::SliderFloat("position y", &light.position.y, -50.0f, 50.0f);
    ImGui::SliderFloat("position z", &light.position.z, -50.0f, 50.0f);

    ImGui::Checkbox("Abilita personaggio", &mainCharacter);

    ImGui::SliderFloat("Character speed", &moveSpeed, 0.000f, 0.005f);

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}


void destroyGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}