#include "VertexBufferLayout.h"

template<> void VertexBufferLayout::Push<float>(unsigned int count)
{
    ASSERT(sizeof(float) == sizeof(GL_FLOAT));
    m_Elements.push_back({ GL_FLOAT, count, GL_FALSE });
    m_Stride += count * sizeof(GL_FLOAT);
}
template<> void VertexBufferLayout::Push<unsigned int>(unsigned int count)
{
    ASSERT(sizeof(unsigned int) == sizeof(GL_UNSIGNED_INT));
    m_Elements.push_back({ GL_UNSIGNED_INT, count, GL_FALSE });
    m_Stride += count * sizeof(GL_UNSIGNED_INT);
}
template<> void VertexBufferLayout::Push<unsigned char>(unsigned int count)
{
    ASSERT(sizeof(unsigned char) == sizeof(GL_UNSIGNED_BYTE));
    m_Elements.push_back({ GL_UNSIGNED_BYTE, count, GL_TRUE });
    m_Stride += count * sizeof(GL_UNSIGNED_BYTE);
}