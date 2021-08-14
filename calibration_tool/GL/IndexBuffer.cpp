#include "IndexBuffer.h"
#include "Renderer.h"
IndexBuffer::IndexBuffer(const unsigned int * data, unsigned int count) :
	m_Count(count)
{
	ASSERT(sizeof(unsigned int) == sizeof(GLuint));
	GLCall(glGenBuffers(1, &m_RenderID));
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RenderID));
	GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, GL_DYNAMIC_DRAW));
}
void IndexBuffer::update(const unsigned int * data, unsigned int count){
    m_Count = count;
    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RenderID));
    GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, GL_DYNAMIC_DRAW));
}

IndexBuffer::~IndexBuffer()
{
	GLCall(glDeleteBuffers(1, &m_RenderID));
}
void IndexBuffer::Bind() const 
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_RenderID));
}
void IndexBuffer::Unbind() const
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
}