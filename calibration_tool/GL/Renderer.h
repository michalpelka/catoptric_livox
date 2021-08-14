#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "Shader.h"
#ifdef _MSC_VER
#define ASSERT(x) if(!(x)) __debugbreak();
#else
#include <signal.h>
#define ASSERT(x) if(!(x))raise(SIGTRAP);
#endif
#define GLCall(x) GLClearError();\
	x;\
	ASSERT(GLLogCall(#x,__FILE__, __LINE__))

void GLClearError();
void GLCheckError();
bool  GLLogCall(const char* function, const char* file, int line);

class Renderer
{
public:
	void Draw(const VertexArray &va, const IndexBuffer& ib, const Shader& shader, GLenum mode, int count = -1) const;
};
