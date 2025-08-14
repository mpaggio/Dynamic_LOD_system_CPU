#include "shaderHandler.h"

/*
* ****************************************************************************************
* --- Funzione per caricare uno shader da file ---
* ****************************************************************************************
*
* PARAMETRI:
* - filepath: percorso del file .glsl che contiene il codice dello shader
*
* OUTPUT: il contenuto del file sottoforma di stringa (std::string)
*/
string loadShaderSource(const char* filepath) {
    ifstream in(filepath); //Crea uno stream di input per leggere il file specificato

    //Gestione dell'errore
    if (!in.is_open()) {
        cerr << "Errore nell'aprire lo shader: " << filepath << endl;
        return "";
    }

    stringstream buffer;
    buffer << in.rdbuf(); //Legge il contenuto del file tutto in una volta
    return buffer.str(); //Converte lo stringstream in una stringa (std::string) e la restituisce
}

/*
* ****************************************************************************************
* --- Compilazione singolo shader ---
* ****************************************************************************************
*
*  PARAMETRI:
*  - path: percorso del file .glsl che contiene il codice dello shader
*  - type: tipo di shader (GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, GL_GEOMETRY_SHADER, ...)
*
*  OUTPUT: l'ID relativo allo shader compilato
*/
unsigned int compileShader(const char* path, GLenum type) {
    string source = loadShaderSource(path); //Apre il file specificato dal path e ne legge il contenuto
    const char* src = source.c_str(); //Converte la stringa in char* (visto che OpenGL usa API C-style)
    unsigned int shader = glCreateShader(type); //Richiede ad OpenGL di creare un oggetto shader del tipo specificato e restituisce il suo ID
    glShaderSource(shader, 1, &src, nullptr); //Associa il sorgente allo shader appena creato (1: numero di stringhe sorgente, &src: puntatore alla stringa, nullptr: la stringa termina con \0)
    glCompileShader(shader); // Compila il codice sorgente nello shader

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success); //Chiede a OpenGL se la compilazione è andata a buon fine

    //Gestione dell'errore
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        cerr << "Errore di compilazione shader (" << path << "):\n" << log << std::endl;
    }

    return shader; //Restituisce l'ID dello shader compilato
}

/*
* ****************************************************************************************
* --- Crea, compila, linga e pulisce un intero shader program ---
* ****************************************************************************************
*
* OUTPUT: l'ID del programma shader completo, pronto per essere usato con glUseProgram
*/
unsigned int createShaderProgram(const char* vs, const char* tcs, const char* tes, const char* gs, const char* fs) {
    unsigned int vertex = compileShader(vs, GL_VERTEX_SHADER); //Vertex Shader
    unsigned int tc = compileShader(tcs, GL_TESS_CONTROL_SHADER); //Tessellation Control Shader
    unsigned int te = compileShader(tes, GL_TESS_EVALUATION_SHADER); //Tessellation Evaluation Shader
    unsigned int geometry = compileShader(gs, GL_GEOMETRY_SHADER); //Geometry Shader
    unsigned int fragment = compileShader(fs, GL_FRAGMENT_SHADER); //Fragment Shader

    unsigned int program = glCreateProgram(); //Crea un nuovo shader program

    //Allega tutti gli shader allo shader program
    glAttachShader(program, vertex);
    glAttachShader(program, tc);
    glAttachShader(program, te);
    glAttachShader(program, geometry);
    glAttachShader(program, fragment);

    glLinkProgram(program); //Collega gli shader insieme in un unico programma eseguibile dalla GPU

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success); //Controlla se il linking è andato a buon fine

    //Gestione dell'errore
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, 512, nullptr, log);
        cerr << "Errore di linking:\n" << log << std::endl;
    }

    //Pulizia degli shader singoli (non rimuove gli shader dal programma, ma li elimina dalla RAM quando non sono più utilizzati)
    glDeleteShader(vertex);
    glDeleteShader(tc);
    glDeleteShader(te);
    glDeleteShader(geometry);
    glDeleteShader(fragment);

    return program; //Restituisce l'ID dello shader program
}

unsigned int createSimpleShaderProgram(const char* vs, const char* fs) {
    unsigned int vertex = compileShader(vs, GL_VERTEX_SHADER); //Vertex Shader
    unsigned int fragment = compileShader(fs, GL_FRAGMENT_SHADER); //Fragment Shader

    unsigned int program = glCreateProgram(); //Crea un nuovo shader program

    //Allega tutti gli shader allo shader program
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);

    glLinkProgram(program); //Collega gli shader insieme in un unico programma eseguibile dalla GPU

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success); //Controlla se il linking è andato a buon fine

    //Gestione dell'errore
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, 512, nullptr, log);
        cerr << "Errore di linking:\n" << log << std::endl;
    }

    //Pulizia degli shader singoli (non rimuove gli shader dal programma, ma li elimina dalla RAM quando non sono più utilizzati)
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    return program; //Restituisce l'ID dello shader program
}

unsigned int createTransformFeedbackShaderProgram(const char* vs, const char* tcs, const char* tes, const char* gs) {
    unsigned int vertex = compileShader(vs, GL_VERTEX_SHADER); //Vertex Shader
    unsigned int tc = compileShader(tcs, GL_TESS_CONTROL_SHADER); //Tessellation Control Shader
    unsigned int te = compileShader(tes, GL_TESS_EVALUATION_SHADER); //Tessellation Evaluation Shader
    unsigned int geometry = compileShader(gs, GL_GEOMETRY_SHADER); //Geometry Shader

    unsigned int program = glCreateProgram(); //Crea un nuovo shader program

    //Allega tutti gli shader allo shader program
    glAttachShader(program, vertex);
    glAttachShader(program, tc);
    glAttachShader(program, te);
    glAttachShader(program, geometry);

    //Transform Feebdack
    const GLchar* feedbackVaryings[] = {
        "characterPositionTransformFeedback",
        "characterNormalTransformFeedback"
    };
    glTransformFeedbackVaryings(program, 2, feedbackVaryings, GL_INTERLEAVED_ATTRIBS);

    glLinkProgram(program); //Collega gli shader insieme in un unico programma eseguibile dalla GPU

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success); //Controlla se il linking è andato a buon fine

    //Gestione dell'errore
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, 512, nullptr, log);
        cerr << "Errore di linking nel transform:\n" << log << std::endl;
    }

    //Pulizia degli shader singoli (non rimuove gli shader dal programma, ma li elimina dalla RAM quando non sono più utilizzati)
    glDeleteShader(vertex);
    glDeleteShader(tc);
    glDeleteShader(te);
    glDeleteShader(geometry);

    return program; //Restituisce l'ID dello shader program
}