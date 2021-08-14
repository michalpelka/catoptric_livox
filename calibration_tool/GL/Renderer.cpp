#include "Renderer.h"
#include <iostream>

void GLClearError()
{
	while (glGetError());
}
void GLCheckError()
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error] : " << error << std::endl;
	}
}
bool  GLLogCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error] : " << error << function << " " << file << ":" << line << std::endl;
		return false;
	}
	return true;
}

void Renderer::Draw(const VertexArray &va, const IndexBuffer& ib, const Shader& shader, GLenum mode, int count) const
{
    shader.Bind();
    va.Bind();
    ib.Bind();
    GLCall(glDrawElements(mode, count>0?count:ib.GetCount(), GL_UNSIGNED_INT, nullptr));
}
