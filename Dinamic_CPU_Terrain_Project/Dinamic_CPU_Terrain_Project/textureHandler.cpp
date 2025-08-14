#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "textureHandler.h"

extern Importer importer;

vector<GLuint> loadAllTextures() {
    const char* paths[] = {
        "./Texture/snowColor.png",
        "./Texture/snowNormal.png",
        "./Texture/rockColor.png",
        "./Texture/rockNormal.png",
        "./Texture/groundColor.png",
        "./Texture/groundNormal.png",
        "./Texture/sandColor.png",
        "./Texture/sandNormal.png"
    };
    vector<GLuint> texturesID;
    int size = sizeof(paths) / sizeof(paths[0]);

    for (int i = 0; i < size; i++) {
        GLuint textureID;
        glGenTextures(1, &textureID);

        int width, height, nrChannels;
        // Carica immagine (RGBA o RGB)
        unsigned char* data = stbi_load(paths[i], &width, &height, &nrChannels, 0);
        if (!data) {
            cerr << "Errore nel caricamento della texture: " << paths[i] << endl;
            continue;
        }

        GLenum format;
        if (nrChannels == 1)
            format = GL_RED;
        else if (nrChannels == 3)
            format = GL_RGB;
        else if (nrChannels == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        // Impostazioni di wrapping e filtering
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        texturesID.push_back(textureID);
    }

    return texturesID;
}

GLuint loadSingleTexture(const string& path) {
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);

    if (!data) {
        cerr << "Failed to load texture: " << path << endl;
        return 0;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    GLenum format = GL_RGB;
    GLint internalFormat = GL_RGB8;

    if (channels == 1) {
        format = GL_RED;
        internalFormat = GL_R8;
    }
    else if (channels == 3) {
        format = GL_RGB;
        internalFormat = GL_RGB8;
    }
    else if (channels == 4) {
        format = GL_RGBA;
        internalFormat = GL_RGBA8;
    }
    else {
        cerr << "Unsupported texture format: " << channels << " channels\n";
        stbi_image_free(data);
        return 0;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);

    return textureID;
}

GLuint loadSkybox() {
    vector<string> faces {
        "Skybox/stars/right.jpg",
        "Skybox/stars/left.jpg",
        "Skybox/stars/top.jpg",
        "Skybox/stars/bottom.jpg",
        "Skybox/stars/front.jpg",
        "Skybox/stars/back.jpg"
    };

    unsigned int textureID;
    glGenTextures(1, &textureID);

    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(0);
    for (unsigned int i = 0; i < faces.size(); i++) {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        
        if (data) {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        }
        else {
            cout << "Cubemap tex failed to load at path: " << faces[i] << endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}

GLuint createFloatTexture2D(int width, int height, const vector<float>& data) {
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, data.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    return textureID;
}