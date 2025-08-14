#include "interactionHandler.h"
#include "cameraHandler.h"

extern float Theta, Phi, cameraSpeed, moveSpeed;
extern bool mouseLocked, lineMode;
extern ViewSetup SetupTelecamera;
extern PerspectiveSetup SetupProspettiva;

void cursor_position_callback(GLFWwindow* window, double xposIn, double yposIn) {
    // Ignora il movimento del mouse se non siamo in modalità esplorazione
    if (!mouseLocked) 
        return; 

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    float sensitivity = 0.05f;
    static bool firstCall = true;
    static float lastX = width / 2.0f;
    static float lastY = height / 2.0f;

    float xpos = float(xposIn);
    float ypos = float(yposIn);

    ypos = height - ypos;

    if (firstCall) {
        lastX = xpos;
        lastY = ypos;
        firstCall = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = ypos - lastY;

    lastX = xpos;
    lastY = ypos;

    xoffset *= sensitivity;
    yoffset *= sensitivity;

    Theta += xoffset;
    Phi += yoffset;

    if (Phi > 89.0f) 
        Phi = 89.0f;

    if (Phi < -89.0f) 
        Phi = -89.0f;

    vec3 newDirection;
    newDirection.x = cos(radians(Theta)) * cos(radians(Phi));
    newDirection.y = sin(radians(Phi));
    newDirection.z = sin(radians(Theta)) * cos(radians(Phi));
    SetupTelecamera.direction = normalize(newDirection);
    SetupTelecamera.target = SetupTelecamera.position + SetupTelecamera.direction;
}


pair<vec3, float> process_input(GLFWwindow* window) {
    static bool mouseWasPressed = false;
    static bool rightMousePressedLastFrame = false;
    float factor = 10.0f;
    vec3 modelMovement = vec3(0.0f);
    float rotationAngle = 0.0f;

    // Verifica se il tasto sinistro è premuto
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (!mouseWasPressed) {
            cameraSpeed = cameraSpeed / factor;
            mouseWasPressed = true;
        }
    }
    else {
        if (mouseWasPressed) {
            cameraSpeed = cameraSpeed * factor;
            mouseWasPressed = false;
        }
    }

    // Toggle modalità controllo visuale / interazione UI
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        if (!rightMousePressedLastFrame) {
            mouseLocked = !mouseLocked;

            if (mouseLocked) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            }
            else {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }

            rightMousePressedLastFrame = true;
        }
    }
    else {
        rightMousePressedLastFrame = false;
    }

    //Pressione "W"
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraForward();
    
    //Pressione "S"
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraBack();
    
    //Pressione "A"
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraLeft();
    
    //Pressione "D"
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraRight();
    
    //Pressione "SPACE"
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        cameraUp();

    //Pressione "L"
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
        lineMode = true;

    //Pressione "F"
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
        lineMode = false;
    
    //Pressione "SHIFT"
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        cameraDown();

    //Pressione "ESC"
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    //Movimento personaggio
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        modelMovement.z -= moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        modelMovement.z += moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        modelMovement.x -= moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        modelMovement.x += moveSpeed;

    if (length(modelMovement) > 0.0001f) {
        rotationAngle = degrees(atan(modelMovement.x, -modelMovement.z));
    }

    return make_pair(modelMovement, -rotationAngle);
}


void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    if (yoffset < 0)
        SetupProspettiva.fovY -= 1; //Rotella del mouse indietro
    else
        SetupProspettiva.fovY += 1;  //Rotella del mouse in avanti

}