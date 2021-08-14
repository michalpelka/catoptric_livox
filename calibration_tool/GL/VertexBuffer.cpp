#include "VertexBuffer.h"
#include "Renderer.h"
VertexBuffer::VertexBuffer(const void * data, unsigned int size):
    data(data), size(size)
{
	GLCall(glGenBuffers(1, &m_RenderID));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RenderID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_DYNAMIC_DRAW));
}

void VertexBuffer::update(const void * data, unsigned int size){
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RenderID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_DYNAMIC_DRAW));
}

void VertexBuffer::update(){
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RenderID));
    GLCall(glBufferSubData(GL_ARRAY_BUFFER, 0, size, data));
}

VertexBuffer::~VertexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RenderID));
}
void VertexBuffer::Bind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_RenderID));
}
void VertexBuffer::Unbind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}

