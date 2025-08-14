#include "noiseHandler.h"

vector<float> textureData;

inline vector<int> MakePermutation() {
	vector<int> p(256);
	for (int i = 0; i < 256; ++i)
		p[i] = i;

	//Utilizza un generatore Mersenne Twister come seme casuale
	random_device rd;
	mt19937 gen(rd());
	shuffle(p.begin(), p.end(), gen);

	p.insert(p.end(), p.begin(), p.end()); //Duplica l'array

	return p;
}

static vector<int> permutation = MakePermutation();

vec2 GetConstantVector(int value) {
	switch (value & 3) {
		case 0: 
			return vec2(1.0f, 1.0f);

		case 1: 
			return vec2(-1.0f, 1.0f);
	
		case 2: 
			return vec2(-1.0f, -1.0f);
	
		default: 
			return vec2(1.0f, -1.0f);
	}
}

float Fade(float t) {
	return ((6 * t - 15) * t + 10) * t * t * t;
}

float Lerp(float t, float a, float b) {
	return a + t * (b - a);
}

float Noise2D(float x, float y) {
	int X = static_cast<int>(floor(x)) & 255;
	int Y = static_cast<int>(floor(y)) & 255;

	float x_decimal = x - floor(x);
	float y_decimal = y - floor(y);

	vec2 topRight = vec2(x_decimal - 1.0, y_decimal - 1.0);
	vec2 topLeft = vec2(x_decimal, y_decimal - 1.0);
	vec2 bottomRight = vec2(x_decimal - 1.0, y_decimal);
	vec2 bottomLeft = vec2(x_decimal, y_decimal);

	// Select a value from the permutation array for each of the 4 corners
	int valueTopRight = permutation[permutation[X + 1] + Y + 1];
	int valueTopLeft = permutation[permutation[X] + Y + 1];
	int valueBottomRight = permutation[permutation[X + 1] + Y];
	int valueBottomLeft = permutation[permutation[X] + Y];

	float dotTopRight = dot(topRight, GetConstantVector(valueTopRight));
	float dotTopLeft = dot(topLeft, GetConstantVector(valueTopLeft));
	float dotBottomRight = dot(bottomRight, GetConstantVector(valueBottomRight));
	float dotBottomLeft = dot(bottomLeft, GetConstantVector(valueBottomLeft));

	float u = Fade(x_decimal);
	float v = Fade(y_decimal);

	return Lerp(u,
		Lerp(v, dotBottomLeft, dotTopLeft),
		Lerp(v, dotBottomRight, dotTopRight)
	);
}

float FractalBrownianMotion(float x, float y, int numOctaves) {
	float result = 0.0f;
	float amplitude = 0.9f; //Definisce l'impatto dell'ottava corrente
	float frequency = 0.005f; //Frequenza bassa (colline e rilievi morbidi), frequenza alta (montagne e rilievi ripidi).
	float gain = 0.5f;
	float lacunarity = 2.0f;

	for (int i = 0; i < numOctaves; ++i) {
		result += amplitude * Noise2D(x * frequency, y * frequency);
		amplitude *= gain;
		frequency *= lacunarity;
	}

	return result;
}

vector<float> generateFBMData(int width, int height, int numOctaves) {
	vector<float> data(width * height);
	float scale = 512.0f * 2.5;

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			float xf = static_cast<float>(x) / width;
			float yf = static_cast<float>(y) / height;
			float value = FractalBrownianMotion(xf * scale, yf * scale, numOctaves);
			data[(y * width) + x] = value;
		}
	}

	return data;
}

float getHeightAt(float x, float z, float terrainSize, int texWidth, int texHeight) {
	// Se la posizione è fuori dal terreno, restituisci altezza 0.0
	if (x < 0.0f || x > terrainSize || z > 0.0f || z < -terrainSize)
		return 0.0f;

	// Converte da world space a UV
	float u = x / terrainSize;
	float v = z / terrainSize;

	// Wrapping in [0, 1] per sicurezza
	u = u - floor(u);
	v = v - floor(v);

	// Conversione a pixel
	int texX = static_cast<int>(u * texWidth);
	int texY = static_cast<int>(v * texHeight);

	// Clamp per sicurezza
	texX = clamp(texX, 0, texWidth - 1);
	texY = clamp(texY, 0, texHeight - 1);

	float value = textureData[texY * texWidth + texX];

	const float HEIGHT_SCALE = 1.5f;
	return value * HEIGHT_SCALE;
}

vec3 getNormalAt(float x, float z, float terrainSize, int texWidth, int texHeight) {
	// Passo di campionamento basato su dimensione terreno e risoluzione heightmap
	float delta = terrainSize / static_cast<float>(std::min(texWidth, texHeight));

	// Campiona le altezze intorno al punto (x,z)
	float hL = getHeightAt(x - delta, z, terrainSize, texWidth, texHeight);
	float hR = getHeightAt(x + delta, z, terrainSize, texWidth, texHeight);
	float hD = getHeightAt(x, z - delta, terrainSize, texWidth, texHeight);
	float hU = getHeightAt(x, z + delta, terrainSize, texWidth, texHeight);

	// Vettori tangenti (dx e dz) lungo la superficie
	vec3 dx = vec3(2.0f * delta, hR - hL, 0.0f);
	vec3 dz = vec3(0.0f, hU - hD, 2.0f * delta);

	// Normale è il cross product dei vettori tangenti
	vec3 normal = normalize(cross(dz, dx));
	return normal;
}