#pragma once
#include <vector>
#include "Renderer.h"
#include <GL/gl.h>
struct VertexBufferElement
{
	unsigned int type;
	unsigned int count;
	unsigned char normalized;
	static unsigned int GetSizeOfType(unsigned int type)
	{
		switch (type)
		{
			case GL_FLOAT: return 4;
			case GL_UNSIGNED_INT: return 4;
			case GL_UNSIGNED_BYTE: return 1;
		}
		ASSERT(false);
		return 0;

	}
};
class VertexBufferLayout
{
public:
	VertexBufferLayout() : m_Stride(0)
	{

	}
	template<typename T> void Push(unsigned int count);
    inline const std::vector<VertexBufferElement>& GetElements() const {return m_Elements;}
	inline unsigned int GetStride() const { return m_Stride; }


private:
	std::vector<VertexBufferElement> m_Elements;
	unsigned int m_Stride;
};

